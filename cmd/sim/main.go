package main

import (
	"github.com/ardalan-sia/envyfree-traffic/pkg/agent"
	"github.com/ardalan-sia/envyfree-traffic/pkg/simulation"
	"github.com/ardalan-sia/envyfree-traffic/pkg/traffic"
)

func main() {
	sim := simulation.NewSimulator()

	// Build graph with explicit IDs
	sim.Graph.AddEdge("E1", "A", "B", 100)
	sim.Graph.AddEdge("E2", "B", "D", 100)
	sim.Graph.AddEdge("E3", "A", "C", 150)
	sim.Graph.AddEdge("E4", "C", "D", 50)

	// Pre-allocate traffic states by edge ID
	for _, edges := range sim.Graph.Edges {
		for _, e := range edges {
			sim.Traffic.Edges[e.ID] = &traffic.EdgeState{
				Length:   e.Length,
				Vehicles: make(map[int]struct{}),
			}
		}
	}

	// Agents
	a1 := &agent.Agent{
		ID:              1,
		CurrentNode:     "A",
		CurrentEdgeID:   "E1",
		DestinationNode: "D",
		Graph:           sim.Graph,
		Traffic:         sim.Traffic,
		Alpha:           20, Vmax: 10, L: 5,
	}
	a2 := &agent.Agent{
		ID:              2,
		CurrentNode:     "A",
		CurrentEdgeID:   "E3",
		DestinationNode: "B",
		Graph:           sim.Graph,
		Traffic:         sim.Traffic,
		Alpha:           20, Vmax: 10, L: 5,
	}

	sim.RegisterAgent(a1)
	sim.RegisterAgent(a2)

	sim.Run()
}
