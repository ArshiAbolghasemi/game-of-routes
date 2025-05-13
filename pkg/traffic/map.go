package traffic

import "sync"

// EdgeState stores dynamic info for a road.
type EdgeState struct {
	Length   float64
	Vehicles map[int]struct{} // agent IDs
	mu       sync.RWMutex
}

// TrafficMap gives thread-safe density queries.
type TrafficMap struct {
	Edges map[string]*EdgeState // key = Edge.ID
}

// NewMap allocates an empty TrafficMap.
func NewMap() *TrafficMap { return &TrafficMap{Edges: make(map[string]*EdgeState)} }

// Density = vehicles / length.
func (tm *TrafficMap) Density(edgeID string) float64 {
	es := tm.Edges[edgeID]
	es.mu.RLock()
	defer es.mu.RUnlock()
	return float64(len(es.Vehicles)) / es.Length
}

func (tm *TrafficMap) AddVehicle(edgeID string, agentID int) {
	es := tm.Edges[edgeID]
	es.mu.Lock()
	es.Vehicles[agentID] = struct{}{}
	es.mu.Unlock()
}

func (tm *TrafficMap) RemoveVehicle(edgeID string, agentID int) {
	es := tm.Edges[edgeID]
	es.mu.Lock()
	delete(es.Vehicles, agentID)
	es.mu.Unlock()
}
