#include "path_finder.h"

#include <queue>


namespace NavMesh {

	void PathFinder::AddPolygons(const std::vector<Polygon>& polygons_to_add, int inflate_by = 0)
	{
		polygons_.clear();
		v_.clear();
		edges_.clear();
		vertex_ids_.clear();
		polygons_.reserve(polygons_to_add.size());
		for (auto const& p : polygons_to_add) {
			polygons_.emplace_back(p.Inflate(inflate_by));
			// Don't add polygons which are not really an obstacle.
			if (polygons_.back().Size() < 1) polygons_.pop_back();
		}

		// Calculate which polygon points are bad, i.e. inside some other polygon.
		polygon_point_is_inside_.resize(polygons_.size());
		for (size_t i = 0; i < polygons_.size(); ++i) {
			int n = polygons_[i].Size();
			polygon_point_is_inside_[i].resize(n);
			for (int k = 0; k < n; ++k) {
				polygon_point_is_inside_[i][k] = false;
				const Point& cur_point = polygons_[i].points_[k];
				for (int j = 0; j < polygons_.size(); ++j) {
					if (i != j && polygons_[j].IsInside(cur_point)) {
						polygon_point_is_inside_[i][k] = true;
						break;
					}
				}
			}
		}

		// Stores tangents from the currently considered point to all the polygons.
		std::vector<std::pair<int, int>> tangents(polygons_.size());

		// Consider all points and add edges from them.
		for (size_t i = 0; i < polygons_.size(); ++i) {
			const auto& cur_poly = polygons_[i];
			int n = cur_poly.Size();
			for (int k = 0; k < n; ++k) {
				if (polygon_point_is_inside_[i][k]) {
					// Current point is strictly inside something.
					// Can't add any edges from it.
					continue;
				}
				const Point& cur_point = cur_poly.points_[k];

				// Pre-calculate all tangents to all polygons to
				// speed-up intersection detection of possible edges.
				for (int j = 0; j < polygons_.size(); ++j) {
					tangents[j] = polygons_[j].GetTangentIds(cur_point);
				}

				// Add sides of polygon from this point. Only one is added here,
				// as the other one is added when the previous point is considered.
				Segment s = Segment(cur_point, cur_poly.points_[(k + 1) % n]);
				if (!polygon_point_is_inside_[i][(k + 1) % n] && CanAddSegment(s, tangents)) {
					AddEdge(GetVertex(s.b), GetVertex(s.e));
				}
				// Add tangents between polygons.
				for (size_t j = i + 1; j < polygons_.size(); ++j) {
					const auto& other_poly = polygons_[j];
					const auto& ids = tangents[j];
					// This point is equal to some vertex.
					// No sense adding edges to this polygon.
					if (ids.first == ids.second) continue;
					// Only consider the segment if it's also tangent to the current polygon.
					if (ids.first >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.first])) {
						s = Segment(cur_poly.points_[k], other_poly.points_[ids.first]);
						if (!polygon_point_is_inside_[j][ids.first] && CanAddSegment(s, tangents)) {
							AddEdge(GetVertex(s.b), GetVertex(s.e));
						}
					}
					if (ids.second >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.second])) {
						s = Segment(cur_poly.points_[k], other_poly.points_[ids.second]);
						if (!polygon_point_is_inside_[j][ids.second] && CanAddSegment(s, tangents)) {
							AddEdge(GetVertex(s.b), GetVertex(s.e));
						}
					}
				}
			}
		}
	}

	void PathFinder::AddExternalPoints(const std::vector<Point>& points_)
	{
		// Remove old points
		for (const auto& p : ext_points_) {
			auto it = vertex_ids_.find(p);
			if (it == vertex_ids_.end()) continue;
			int id = it->second;
			free_vertices_.push_back(id);
			// Remove all edges from and to the point.
			for (const auto& e : edges_[id]) {
				int u = e.first;
				if (u == id) continue;
				size_t i;
				for (i = 0; i < edges_[u].size(); ++i) {
					if (edges_[u][i].first == id) {
						edges_[u][i] = edges_[u].back();
						edges_[u].resize(edges_[u].size() - 1);
						break;
					}
				}
			}
			edges_[id].clear();
			vertex_ids_.erase(it);
		}

		ext_points_ = points_;

		std::vector<std::pair<int, int>> tangents(polygons_.size());
		std::vector<bool> point_is_inside(points_.size(), false);

		for (int i = 0; i < points_.size(); ++i) {
			for (int j = 0; j < polygons_.size(); ++j) {
				if (polygons_[j].IsInside(points_[i])) {
					point_is_inside[i] = true;
					break;
				}
			}
		}

		for (int i = 0; i < points_.size(); ++i) {
			const auto& cur_point = points_[i];
			// Points inside of obstacle are bad.
			// Also, Polygon::Intersect assumes endpoints of the segment
			// are not inside of the polygon.
			if (point_is_inside[i]) continue;
			for (int j = 0; j < polygons_.size(); ++j) {
				tangents[j] = polygons_[j].GetTangentIds(cur_point);
			}
			// Add edges between external points.
			for (size_t j = i + 1; j < points_.size(); ++j) {
				if (!point_is_inside[j] && CanAddSegment(Segment(cur_point, points_[j]), tangents)) {
					AddEdge(GetVertex(cur_point), GetVertex(points_[j]));
				}
			}
			// Add tangents to polygons.
			for (int j = 0; j < polygons_.size(); ++j) {
				const auto& ids = tangents[j];
				if (ids.first == -1 || ids.second == -1 || ids.first == ids.second)
					continue;
				const Point& other_point1 = polygons_[j].points_[ids.first];
				if (!polygon_point_is_inside_[j][ids.first] &&
					CanAddSegment(Segment(cur_point, other_point1), tangents)) {
					AddEdge(GetVertex(cur_point), GetVertex(other_point1));
				}
				const Point& other_point2 = polygons_[j].points_[ids.second];
				if (!polygon_point_is_inside_[j][ids.second] &&
					CanAddSegment(Segment(cur_point, other_point2), tangents)) {
					AddEdge(GetVertex(cur_point), GetVertex(other_point2));
				}
			}
		}
	}

	std::vector<Point> PathFinder::GetPath(const Point& start_coord, const Point& dest_coord)
	{
		int start = GetVertex(start_coord);
		int dest = GetVertex(dest_coord);

		if (start == dest) return { start_coord };

		// Run A*.
		std::vector<int> prev(v_.size(), -1);
		std::vector<double> dist(v_.size(), -1.0);
		std::vector<double> est(v_.size(), -1.0);
		std::vector<bool> done(v_.size(), false);
		std::priority_queue<std::pair<double, int>> queue;

		if (polygons_.size() > 0) {
			start = start;
		}

		dist[start] = 0;
		est[start] = (v_[dest] - v_[start]).Len();
		queue.push(std::make_pair(-est[start], start));
		while (!queue.empty()) {
			int bst = queue.top().second;
			queue.pop();
			if (done[bst]) continue;
			done[bst] = true;
			if (bst == dest) break;
			for (const auto& e : edges_[bst]) {
				if (dist[e.first] < 0 || dist[e.first] > dist[bst] + e.second) {
					dist[e.first] = dist[bst] + e.second;
					est[e.first] = dist[e.first] + (v_[dest] - v_[e.first]).Len();
					// Put negative distance esimate since the queue is for maximum and we 
					// need minimum.
					queue.push(std::make_pair(-est[e.first], e.first));
					prev[e.first] = bst;
				}
			}
		}

		if (prev[dest] == -1) return {};

		std::vector<Point> res;
		int u = dest;
		while (u != start) {
			res.push_back(v_[u]);
			u = prev[u];
		}
		res.push_back(v_[start]);
		std::reverse(res.begin(), res.end());
		return res;
	}

	std::vector<Segment> PathFinder::GetEdgesForDebug() const
	{
		std::vector<Segment> res;
		for (int i = 0; i < (int)edges_.size(); ++i) {
			for (const auto& e : edges_[i]) {
				int j = e.first;
				if (j > i) {
					res.push_back(Segment(v_[i], v_[j]));
				}
			}
		}
		return res;
	}

	int PathFinder::GetVertex(const Point& c)
	{
		auto it = vertex_ids_.find(c);
		if (it != vertex_ids_.end()) {
			return it->second;
		}
		if (free_vertices_.empty()) {
			vertex_ids_[c] = (int)v_.size();
			v_.push_back(c);
			edges_.push_back({});
			return (int)v_.size() - 1;
		}
		else {
			int node = free_vertices_.back();
			free_vertices_.pop_back();
			v_[node] = c;
			vertex_ids_[c] = node;
			return node;
		}
	}

	void PathFinder::AddEdge(int be, int en)
	{
		double dst = (v_[be] - v_[en]).Len();
		edges_[be].push_back(std::make_pair(en, dst));
		edges_[en].push_back(std::make_pair(be, dst));
	}

	bool PathFinder::CanAddSegment(const Segment& s, const std::vector<std::pair<int, int>>& tangents)
	{
		for (size_t i = 0; i < polygons_.size(); ++i) {
			if (polygons_[i].Intersects(s, tangents[i])) return false;
		}
		return true;
	}




}