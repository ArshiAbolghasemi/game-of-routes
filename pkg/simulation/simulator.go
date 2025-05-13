package simulation

import (
	"sync"

	"github.com/ardalan-sia/envyfree-traffic/pkg/agent"
	"github.com/ardalan-sia/envyfree-traffic/pkg/graph"
	"github.com/ardalan-sia/envyfree-traffic/pkg/traffic"
)

type Simulator struct {
	Graph   *graph.Graph
	Traffic *traffic.TrafficMap
	Agents  []*agent.Agent
}

func NewSimulator() *Simulator {
	return &Simulator{
		Graph:   graph.New(),
		Traffic: traffic.NewMap(),
	}
}

func (s *Simulator) RegisterAgent(a *agent.Agent) {
	s.Agents = append(s.Agents, a)
	s.Traffic.AddVehicle(a.CurrentEdgeID, a.ID)
}

func (s *Simulator) Run() {
	var wg sync.WaitGroup
	done := make(chan int, len(s.Agents))
	for _, a := range s.Agents {
		wg.Add(1)
		go func(ag *agent.Agent) {
			defer wg.Done()
			ag.Run(done)
		}(a)
	}

	wg.Wait()
	close(done)

}
