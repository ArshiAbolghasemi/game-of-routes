package graph

import (
	"container/heap"
	"math"
)

// Edge represents a directed road.
type Edge struct {
	ID     string
	From   string
	To     string
	Length float64
	Wight  float64
}

// Graph is a simple directed graph.
type Graph struct {
	Nodes map[string]struct{}
	Edges map[string][]*Edge // key = from node
}

// New returns an empty graph.
func New() *Graph {
	return &Graph{
		Nodes: make(map[string]struct{}),
		Edges: make(map[string][]*Edge),
	}
}

// AddEdge inserts a directed edge.
func (g *Graph) AddEdge(id, from, to string, length float64) {
	g.Nodes[from], g.Nodes[to] = struct{}{}, struct{}{}
	e := &Edge{ID: id, From: from, To: to, Length: length}
	g.Edges[from] = append(g.Edges[from], e)
}

// Neighbors returns outgoing edges.
func (g *Graph) Neighbors(node string) []*Edge { return g.Edges[node] }

// Dijkstra with custom weightFn.
func (g *Graph) Dijkstra(start, goal string, weight func(*Edge) float64) ([]string, float64) {
	dist := make(map[string]float64)
	prev := make(map[string]string)
	for n := range g.Nodes {
		dist[n] = math.Inf(1)
	}
	dist[start] = 0

	pq := &priorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &item{node: start, priority: 0})

	for pq.Len() > 0 {
		u := heap.Pop(pq).(*item).node
		if u == goal {
			break
		}
		for _, e := range g.Neighbors(u) {
			alt := dist[u] + weight(e)
			if alt < dist[e.To] {
				dist[e.To] = alt
				prev[e.To] = u
				heap.Push(pq, &item{node: e.To, priority: alt})
			}
		}
	}

	// reconstruct path of nodes
	var path []string
	for u := goal; u != ""; u = prev[u] {
		path = append([]string{u}, path...)
	}
	return path, dist[goal]
}

// ---------- internal PQ ----------
type item struct {
	node     string
	priority float64
}
type priorityQueue []*item

func (pq priorityQueue) Len() int            { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool  { return pq[i].priority < pq[j].priority }
func (pq priorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i] }
func (pq *priorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*item)) }
func (pq *priorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	it := old[n-1]
	*pq = old[:n-1]
	return it
}
