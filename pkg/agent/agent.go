package agent

import (
	"fmt"
	"time"

	"github.com/ardalan-sia/envyfree-traffic/pkg/graph"
	"github.com/ardalan-sia/envyfree-traffic/pkg/traffic"
)

// Agent models one vehicle.
type Agent struct {
	ID int

	CurrentNode     string
	CurrentEdgeID   string
	DestinationNode string

	Graph   *graph.Graph
	Traffic *traffic.TrafficMap

	Alpha, Vmax, L float64
}

// Run executes agent logic until arrival.
func (a *Agent) Run(done chan<- int) {
	for {
		if a.CurrentNode == a.DestinationNode {
			fmt.Printf("[Agent %d] ✅ Reached destination %s\n", a.ID, a.DestinationNode)
			a.Traffic.RemoveVehicle(a.CurrentEdgeID, a.ID)
			done <- a.ID
			return
		}

		fmt.Printf("[Agent %d] 📍 At node: %s | Current edge: %s\n", a.ID, a.CurrentNode, a.CurrentEdgeID)

		// Define the paper-based weight function
		weightFn := func(e *graph.Edge) float64 {
			d := a.Traffic.Density(e.ID)
			cruise := ((200 - a.Alpha) / 100.0) * e.Length
			brake := 2 * d * a.L
			total := (cruise + brake) / a.Vmax
			fmt.Printf("[Agent %d] → Edge %s (%s→%s) | Len=%.0f, Density=%.2f | Weight=%.2f\n",
				a.ID, e.ID, e.From, e.To, e.Length, d, total)
			return total
		}

		// Recompute path from current node to destination
		path, cost := a.Graph.Dijkstra(a.CurrentNode, a.DestinationNode, weightFn)
		if len(path) < 2 {
			fmt.Printf("[Agent %d] ❌ No path found from %s to %s\n", a.ID, a.CurrentNode, a.DestinationNode)
			done <- a.ID
			return
		}
		fmt.Printf("[Agent %d] 🗺️ Planned path: %v (total cost: %.2f)\n", a.ID, path, cost)

		nextNode := path[1]

		// Find corresponding edge object
		var nextEdge *graph.Edge
		for _, e := range a.Graph.Neighbors(a.CurrentNode) {
			if e.To == nextNode {
				nextEdge = e
				break
			}
		}
		if nextEdge == nil {
			fmt.Printf("[Agent %d] ❌ No edge from %s to %s\n", a.ID, a.CurrentNode, nextNode)
			done <- a.ID
			return
		}

		fmt.Printf("[Agent %d] 🚗 Moving to: %s via edge %s\n", a.ID, nextNode, nextEdge.ID)

		// Update traffic map
		a.Traffic.RemoveVehicle(a.CurrentEdgeID, a.ID)
		a.Traffic.AddVehicle(nextEdge.ID, a.ID)

		// Update agent's position
		a.CurrentEdgeID = nextEdge.ID
		a.CurrentNode = nextEdge.To

		time.Sleep(500 * time.Millisecond) // simulate travel time
	}
}
