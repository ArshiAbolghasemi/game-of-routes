package main

import (
	"container/heap"
	"fmt"
	"math"
	"sync"
	"time"
)

// Constants
const (
	MaxVelocity   = 20.0 // maximum velocity for any agent
	VehicleLength = 2.0  // length of a vehicle
	Alpha         = 15   // percentage of road where deceleration begins
)

// Graph represents the road network
type Graph struct {
	Nodes      map[int]*Node
	Edges      map[int]*Edge
	mutex      sync.RWMutex
	agentCount int
}

// Node represents an intersection in the graph
type Node struct {
	ID        int
	OutEdges  []*Edge
	InEdges   []*Edge
	mutex     sync.RWMutex
	WaitQueue []*Agent // Queue of agents waiting at this intersection
}

// Edge represents a road in the graph
type Edge struct {
	ID       int
	Source   *Node
	Target   *Node
	Length   float64  // Length of the road
	Agents   []*Agent // Agents currently on this edge
	mutex    sync.RWMutex
	capacity int // Maximum number of agents on this edge
}

// Agent represents a vehicle in the traffic system
type Agent struct {
	ID            int
	CurrentEdge   *Edge
	Position      float64 // Position on the current edge
	Velocity      float64
	Acceleration  float64
	Source        *Node
	Destination   *Node
	Path          []*Edge // Planned path to destination
	ExpectedTime  float64 // Expected time to reach destination
	ActualTime    float64 // Actual time spent
	EnvyFreeRatio float64 // Ratio used to measure envy-freeness
	State         AgentState
	mutex         sync.RWMutex
}

// AgentState represents the current state of an agent
type AgentState int

const (
	Traveling AgentState = iota
	WaitingAtIntersection
	Arrived
)

// Create a new graph
func NewGraph() *Graph {
	return &Graph{
		Nodes: make(map[int]*Node),
		Edges: make(map[int]*Edge),
	}
}

// Add a node to the graph
func (g *Graph) AddNode(id int) *Node {
	g.mutex.Lock()
	defer g.mutex.Unlock()
	
	if _, exists := g.Nodes[id]; !exists {
		g.Nodes[id] = &Node{
			ID:        id,
			OutEdges:  []*Edge{},
			InEdges:   []*Edge{},
			WaitQueue: []*Agent{},
		}
	}
	return g.Nodes[id]
}

// Add an edge to the graph
func (g *Graph) AddEdge(id, sourceID, targetID int, length float64) *Edge {
	g.mutex.Lock()
	defer g.mutex.Unlock()
	
	sourceNode := g.AddNode(sourceID)
	targetNode := g.AddNode(targetID)
	
	edge := &Edge{
		ID:       id,
		Source:   sourceNode,
		Target:   targetNode,
		Length:   length,
		Agents:   []*Agent{},
		capacity: int(length / (2 * VehicleLength)), // Simple capacity calculation
	}
	
	g.Edges[id] = edge
	sourceNode.OutEdges = append(sourceNode.OutEdges, edge)
	targetNode.InEdges = append(targetNode.InEdges, edge)
	
	return edge
}

// Calculate edge density
func (e *Edge) Density() float64 {
	e.mutex.RLock()
	defer e.mutex.RUnlock()
	
	if e.Length == 0 {
		return 0
	}
	return float64(len(e.Agents)) / e.Length
}

// Calculate edge weight based on the paper's formula
func (e *Edge) Weight() float64 {
	density := e.Density()
	// W(e) = ((200-α)/100) * Len(e) + 2*d(e)*l / v
	return ((200 - Alpha) / 100) * e.Length + 2*density*VehicleLength/MaxVelocity
}

// Calculate non-linear car-following model parameters
func calculateLambda() float64 {
	// λ = v_max / ln(1/(ρ_max * L))
	maxDensity := 1.0 / VehicleLength // Maximum density (bumper to bumper)
	return MaxVelocity / math.Log(1.0/(maxDensity*VehicleLength))
}

// Update agent velocity based on non-linear car-following model
func (a *Agent) UpdateVelocity() {
	a.mutex.Lock()
	defer a.mutex.Unlock()
	
	if a.CurrentEdge == nil {
		a.Velocity = 0
		return
	}
	
	a.CurrentEdge.mutex.RLock()
	agents := a.CurrentEdge.Agents
	a.CurrentEdge.mutex.RUnlock()
	
	// Find the agent in front
	agentInFront := (*Agent)(nil)
	minDistance := math.Inf(1)
	
	for _, other := range agents {
		if other.ID == a.ID {
			continue
		}
		
		other.mutex.RLock()
		otherPos := other.Position
		other.mutex.RUnlock()
		
		if otherPos > a.Position {
			distance := otherPos - a.Position
			if distance < minDistance {
				minDistance = distance
				agentInFront = other
			}
		}
	}
	
	// If no agent in front or at max velocity, maintain max velocity
	if agentInFront == nil {
		// Check if we need to slow down for an intersection
		remainingDist := a.CurrentEdge.Length - a.Position
		if remainingDist <= (Alpha/100.0)*a.CurrentEdge.Length {
			// In the deceleration zone
			decelDist := (Alpha / 100.0) * a.CurrentEdge.Length
			ratio := remainingDist / decelDist
			a.Velocity = MaxVelocity * ratio
		} else {
			a.Velocity = MaxVelocity
		}
		return
	}
	
	// Apply non-linear car-following model
	lambda := calculateLambda()
	agentInFront.mutex.RLock()
	distance := agentInFront.Position - a.Position - VehicleLength
	agentInFront.mutex.RUnlock()
	
	if distance <= 0 {
		a.Velocity = 0 // Safety: stop if too close
	} else {
		// v_i(t) = λ * ln|x_i(t) - x_(i-1)(t)| - λ * ln|L|
		a.Velocity = lambda * math.Log(distance) - lambda*math.Log(VehicleLength)
		if a.Velocity > MaxVelocity {
			a.Velocity = MaxVelocity
		}
		if a.Velocity < 0 {
			a.Velocity = 0
		}
	}
}

// Dijkstra's algorithm for finding shortest path
type PathNode struct {
	node   *Node
	cost   float64
	parent *PathNode
	edge   *Edge
}

type PriorityQueue []*PathNode

func (pq PriorityQueue) Len() int            { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool  { return pq[i].cost < pq[j].cost }
func (pq PriorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i] }
func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*PathNode)) }
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

// Find shortest path using Dijkstra's algorithm
func (a *Agent) FindPath() []*Edge {
	source := a.Source
	destination := a.Destination
	
	// Initialize distance map and visited set
	distances := make(map[int]float64)
	visited := make(map[int]bool)
	previous := make(map[int]*PathNode)
	
	// Initialize priority queue
	pq := make(PriorityQueue, 0)
	heap.Init(&pq)
	
	// Set initial node distance to 0
	heap.Push(&pq, &PathNode{node: source, cost: 0, parent: nil, edge: nil})
	distances[source.ID] = 0
	
	for pq.Len() > 0 {
		// Get node with minimum distance
		currentNode := heap.Pop(&pq).(*PathNode)
		
		// If we've reached the destination, reconstruct the path
		if currentNode.node.ID == destination.ID {
			path := []*Edge{}
			for currentNode.parent != nil {
				path = append([]*Edge{currentNode.edge}, path...)
				currentNode = currentNode.parent
			}
			return path
		}
		
		// Skip if already visited
		if visited[currentNode.node.ID] {
			continue
		}
		
		// Mark as visited
		visited[currentNode.node.ID] = true
		
		// Check all neighbors
		for _, edge := range currentNode.node.OutEdges {
			neighbor := edge.Target
			
			// Calculate weight based on current traffic conditions
			weight := edge.Weight()
			newDist := distances[currentNode.node.ID] + weight
			
			// If we found a better path, update
			if dist, ok := distances[neighbor.ID]; !ok || newDist < dist {
				distances[neighbor.ID] = newDist
				previous[neighbor.ID] = &PathNode{
					node:   currentNode.node,
					cost:   newDist,
					parent: currentNode,
					edge:   edge,
				}
				
				// Add to priority queue
				heap.Push(&pq, &PathNode{
					node:   neighbor,
					cost:   newDist,
					parent: currentNode,
					edge:   edge,
				})
			}
		}
	}
	
	// No path found
	return nil
}

// Calculate expected travel time based on current path
func (a *Agent) CalculateExpectedTime() float64 {
	var totalTime float64
	
	// Add time for current edge
	if a.CurrentEdge != nil {
		a.CurrentEdge.mutex.RLock()
		remainingDist := a.CurrentEdge.Length - a.Position
		a.CurrentEdge.mutex.RUnlock()
		
		// Simple time calculation based on remaining distance and velocity
		totalTime += remainingDist / MaxVelocity
	}
	
	// Add time for each edge in the path
	for _, edge := range a.Path {
		totalTime += edge.Weight()
	}
	
	return totalTime
}

// Create a new agent
func (g *Graph) CreateAgent(id int, sourceID, destID int) *Agent {
	g.mutex.Lock()
	g.agentCount++
	g.mutex.Unlock()
	
	agent := &Agent{
		ID:           id,
		Source:       g.Nodes[sourceID],
		Destination:  g.Nodes[destID],
		Velocity:     0,
		Acceleration: 0,
		Position:     0,
		State:        WaitingAtIntersection,
	}
	
	// Add agent to source node's wait queue
	agent.Source.mutex.Lock()
	agent.Source.WaitQueue = append(agent.Source.WaitQueue, agent)
	agent.Source.mutex.Unlock()
	
	// Calculate initial path
	agent.Path = agent.FindPath()
	
	// Calculate expected time
	agent.ExpectedTime = agent.CalculateExpectedTime()
	
	return agent
}

// Move agent along its path
func (a *Agent) Move(timeStep float64) bool {
	a.mutex.Lock()
	defer a.mutex.Unlock()
	
	// If already arrived, do nothing
	if a.State == Arrived {
		return false
	}
	
	// If waiting at intersection, try to enter the next road
	if a.State == WaitingAtIntersection {
		if len(a.Path) == 0 {
			// No path, something is wrong
			return false
		}
		
		nextEdge := a.Path[0]
		
		// Check if edge has capacity
		nextEdge.mutex.Lock()
		if len(nextEdge.Agents) < nextEdge.capacity {
			// Enter the edge
			a.CurrentEdge = nextEdge
			a.Position = 0
			a.Velocity = MaxVelocity // Start with max velocity
			a.State = Traveling
			
			// Add agent to the edge
			nextEdge.Agents = append(nextEdge.Agents, a)
			
			// Remove first edge from path
			a.Path = a.Path[1:]
			
			nextEdge.mutex.Unlock()
		} else {
			// Edge is full, wait
			nextEdge.mutex.Unlock()
			return false
		}
	}
	
	// If traveling, update position
	if a.State == Traveling {
		a.UpdateVelocity()
		
		// Update position based on velocity
		newPosition := a.Position + a.Velocity*timeStep
		
		// Check if reached the end of the edge
		if newPosition >= a.CurrentEdge.Length {
			// Remove from current edge
			a.CurrentEdge.mutex.Lock()
			for i, agent := range a.CurrentEdge.Agents {
				if agent.ID == a.ID {
					a.CurrentEdge.Agents = append(a.CurrentEdge.Agents[:i], a.CurrentEdge.Agents[i+1:]...)
					break
				}
			}
			a.CurrentEdge.mutex.Unlock()
			
			// If there are more edges in the path, move to the next intersection
			if len(a.Path) > 0 {
				// Recalculate path at intersection to ensure envy-freeness
				a.Path = a.FindPath()
				
				// Update expected time
				a.ExpectedTime = a.CalculateExpectedTime()
				
				// Add to the next intersection's wait queue
				nextNode := a.CurrentEdge.Target
				nextNode.mutex.Lock()
				nextNode.WaitQueue = append(nextNode.WaitQueue, a)
				nextNode.mutex.Unlock()
				
				a.State = WaitingAtIntersection
			} else {
				// Arrived at destination
				a.State = Arrived
				a.ActualTime += timeStep * (newPosition - a.Position) / a.Velocity
				return true
			}
		} else {
			// Continue on current edge
			a.Position = newPosition
			a.ActualTime += timeStep
		}
	}
	
	return false
}

// Calculate envy-free ratio
func (a *Agent) CalculateEnvyFreeRatio(otherAgents []*Agent) {
	a.mutex.Lock()
	defer a.mutex.Unlock()
	
	// Envy-free ratio is defined as the ratio of expected time to actual time
	// A ratio close to 1 indicates the system is envy-free
	if a.ActualTime > 0 {
		a.EnvyFreeRatio = a.ExpectedTime / a.ActualTime
	}
}

// Process intersections - ensure one agent enters at a time
func (n *Node) ProcessIntersection() {
	n.mutex.Lock()
	defer n.mutex.Unlock()
	
	if len(n.WaitQueue) == 0 {
		return
	}
	
	// Get the first agent in the queue
	agent := n.WaitQueue[0]
	n.WaitQueue = n.WaitQueue[1:]
	
	// Let the agent attempt to move (enter the next road)
	agent.Move(0.1)
}

// Simulation represents the entire traffic system simulation
type Simulation struct {
	Graph      *Graph
	Agents     []*Agent
	TimeStep   float64
	TotalTime  float64
	Running    bool
	wg         sync.WaitGroup
	mutex      sync.RWMutex
}

// Create a new simulation
func NewSimulation(g *Graph, timeStep float64) *Simulation {
	return &Simulation{
		Graph:    g,
		Agents:   []*Agent{},
		TimeStep: timeStep,
		Running:  false,
	}
}

// Add an agent to the simulation
func (s *Simulation) AddAgent(a *Agent) {
	s.mutex.Lock()
	defer s.mutex.Unlock()
	s.Agents = append(s.Agents, a)
}

// Run the simulation
func (s *Simulation) Run(duration float64) {
	s.mutex.Lock()
	if s.Running {
		s.mutex.Unlock()
		return
	}
	s.Running = true
	s.mutex.Unlock()
	
	// Start agent goroutines
	for _, agent := range s.Agents {
		s.wg.Add(1)
		go s.runAgent(agent)
	}
	
	// Start node goroutines for intersections
	for _, node := range s.Graph.Nodes {
		s.wg.Add(1)
		go s.runNode(node)
	}
	
	// Monitor simulation time
	for s.TotalTime < duration {
		time.Sleep(time.Duration(s.TimeStep * 1000) * time.Millisecond)
		
		s.mutex.Lock()
		s.TotalTime += s.TimeStep
		s.mutex.Unlock()
		
		// Check if all agents have arrived
		allArrived := true
		for _, agent := range s.Agents {
			agent.mutex.RLock()
			if agent.State != Arrived {
				allArrived = false
			}
			agent.mutex.RUnlock()
			
			if !allArrived {
				break
			}
		}
		
		if allArrived {
			break
		}
	}
	
	s.mutex.Lock()
	s.Running = false
	s.mutex.Unlock()
	
	// Signal all goroutines to stop
	s.wg.Wait()
	
	// Calculate and display envy-free ratios
	s.calculateEnvyFreeRatios()
}

// Run an agent in its own goroutine
func (s *Simulation) runAgent(agent *Agent) {
	defer s.wg.Done()
	
	ticker := time.NewTicker(time.Duration(s.TimeStep * 1000) * time.Millisecond)
	defer ticker.Stop()
	
	for range ticker.C {
		s.mutex.RLock()
		running := s.Running
		s.mutex.RUnlock()
		
		if !running {
			return
		}
		
		agent.mutex.RLock()
		state := agent.State
		agent.mutex.RUnlock()
		
		if state == Arrived {
			return
		}
		
		// Move the agent
		agent.Move(s.TimeStep)
	}
}

// Run a node in its own goroutine to process intersections
func (s *Simulation) runNode(node *Node) {
	defer s.wg.Done()
	
	ticker := time.NewTicker(time.Duration(s.TimeStep * 500) * time.Millisecond)
	defer ticker.Stop()
	
	for range ticker.C {
		s.mutex.RLock()
		running := s.Running
		s.mutex.RUnlock()
		
		if !running {
			return
		}
		
		// Process the intersection
		node.ProcessIntersection()
	}
}

// Calculate envy-free ratios for all agents
func (s *Simulation) calculateEnvyFreeRatios() {
	for _, agent := range s.Agents {
		agent.CalculateEnvyFreeRatio(s.Agents)
	}
}

// Print simulation results
func (s *Simulation) PrintResults() {
	fmt.Println("Simulation Results:")
	fmt.Printf("Total Time: %.2f seconds\n", s.TotalTime)
	fmt.Println("Agent Results:")
	
	for _, agent := range s.Agents {
		fmt.Printf("Agent %d: Expected Time: %.2f, Actual Time: %.2f, Envy-Free Ratio: %.2f, State: %v\n",
			agent.ID, agent.ExpectedTime, agent.ActualTime, agent.EnvyFreeRatio, agent.State)
	}
	
	// Calculate average envy-free ratio
	var totalRatio float64
	var arrivedCount int
	
	for _, agent := range s.Agents {
		if agent.State == Arrived {
			totalRatio += agent.EnvyFreeRatio
			arrivedCount++
		}
	}
	
	if arrivedCount > 0 {
		avgRatio := totalRatio / float64(arrivedCount)
		fmt.Printf("Average Envy-Free Ratio: %.2f\n", avgRatio)
	}
}

// Example usage of the simulation
func main() {
	// Create a new graph
	graph := NewGraph()
	
	// Add nodes (intersections)
	for i := 1; i <= 6; i++ {
		graph.AddNode(i)
	}
	
	// Add edges (roads)
	graph.AddEdge(1, 1, 2, 100)
	graph.AddEdge(2, 2, 3, 150)
	graph.AddEdge(3, 3, 6, 200)
	graph.AddEdge(4, 1, 4, 120)
	graph.AddEdge(5, 4, 5, 130)
	graph.AddEdge(6, 5, 6, 180)
	graph.AddEdge(7, 2, 5, 100)
	graph.AddEdge(8, 4, 3, 110)
	
	// Create a new simulation
	sim := NewSimulation(graph, 0.1) // 0.1 second time step
	
	// Add agents
	for i := 1; i <= 10; i++ {
		// Create agents with different source and destination nodes
		sourceID := 1
		destID := 6
		agent := graph.CreateAgent(i, sourceID, destID)
		sim.AddAgent(agent)
		
		// Add a small delay between adding agents
		time.Sleep(500 * time.Millisecond)
	}
	
	// Run the simulation for 60 seconds
	fmt.Println("Starting simulation...")
	sim.Run(60)
	
	// Print results
	sim.PrintResults()
}
