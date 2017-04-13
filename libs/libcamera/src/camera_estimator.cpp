
#include "camera_estimator.h"
#include <queue>

#include "config.h"
#include "camera.h"
#include "match_info.h"
#include "incremental_bundle_adjuster.h"

using namespace std;


int STRAIGHTEN =1;

const int MULTIPASS_BA = 1;
	//# 0: only perform one-pass bundle adjustment for all images and connections (fast)
	//# 1: perform BA for each image added (suggested)
	//# 2: perform BA for each connection found (best quality, slow)
    

CameraEstimator::CameraEstimator(
		std::vector<std::vector<MatchInfo>>& matches,
		const std::vector<Shape2D>& image_shapes) :
		n(matches.size()),
		matches(matches),
		shapes(image_shapes),
		cameras(matches.size())
	{ 
		//m_assert(matches.size() == shapes.size()); 
	}

CameraEstimator::~CameraEstimator() = default;

vector<Camera> CameraEstimator::estimate() 
{
	//GuardedTimer tm("Estimate Camera");
	{ // assign an initial focal length
		double focal = Camera::estimate_focal(matches);
		if (focal > 0) {
			for (auto& c : cameras)
				c.focal = focal;
			printf("Estimated focal: %lf\n", focal);
		} else {
			printf("Cannot estimate focal. Will use a naive one.\n");
			for(int i=0;i<n;++i) // hack focal
				cameras[i].focal = (shapes[i].w + shapes[i].h) * 0.5;
		}
	}

	IncrementalBundleAdjuster iba(shapes, cameras);
	vector<bool> vst(n, false);
	traverse(
		[&](int node) {
			// set the starting point to identity
			cameras[node].R = Homography::I();
			cameras[node].ppx = shapes[node].halfw();
			cameras[node].ppy = shapes[node].halfh();
		},
		[&](int now, int next) {
			printf("Best edge from %d to %d\n", now, next);
			auto Kfrom = cameras[now].K();
			auto Kto = cameras[next].K();
			auto Hinv = matches[now][next].homo;	// from next to now
			Kfrom[2] = Kfrom[5] = 0;		// set K to zero, because homo operates on zero-based index
			auto Mat = Kfrom.inverse() * Hinv * Kto;
			// this is camera extrincis R, i.e. going from identity to this image
			cameras[next].R = (cameras[now].Rinv() * Mat).transpose();
			cameras[next].ppx = shapes[next].halfw();
			cameras[next].ppy = shapes[next].halfh();

			//cameras[next] = cameras[now];	// initialize by the last camera. also good

			if (MULTIPASS_BA > 0) {
				// add next to BA
				vst[now] = vst[next] = true;
				for(int i=0;i<n;++i)
					if (vst[i] && i != next) 
					{
						auto& m = matches[next][i];
						if (m.match.size() && m.confidence > 0) {
							iba.add_match(i, next, m);
							if (MULTIPASS_BA == 2)
								iba.optimize();
						}
					}
				if (MULTIPASS_BA == 1)
					iba.optimize();
			}
		});

	if (MULTIPASS_BA == 0) 
	{		// optimize everything together
		for(int i=1;i<n;++i) 
			for(int j=0;j<i;++j) 
			{
				auto& m = matches[j][i];
				if (m.match.size() && m.confidence > 0)
					iba.add_match(i, j, m);
			}
		iba.optimize();
	}

	if (STRAIGHTEN) Camera::straighten(cameras);
	return cameras;
}

void CameraEstimator::traverse(
		function<void(int)> callback_node,
		function<void(int, int)> callback_edge) {
	struct Edge {
		int v1, v2;
		float weight;
		Edge(int a, int b, float v):v1(a), v2(b), weight(v) {}
		bool operator < (const Edge& r) const { return weight < r.weight;	}
	};
	// choose a starting point
	Edge best_edge{-1, -1, 0};
	for(int i=0;i<n;++i) 
		for(int j=i+1;j<n;++j) 
		{
			auto& m = matches[i][j];
			if (m.confidence > best_edge.weight)
				best_edge = Edge{i, j, m.confidence};
		}
	if (best_edge.v1 == -1)
	{
		printf("No connected images are found!");
		exit(1);
	}
	callback_node(best_edge.v1);

	priority_queue<Edge> q;
	vector<bool> vst(n, false);

	auto enqueue_edges_from = [&](int from) {
		for(int i=0;i<n;++i) 
		if (i != from && !vst[i]) 
		{
			auto& m = matches[from][i];
			if (m.confidence > 0)
				q.emplace(from, i, m.confidence);
		}
	};

	vst[best_edge.v1] = true;
	enqueue_edges_from(best_edge.v1);
	int cnt = 1;
	while (q.size()) {
		do {
			best_edge = q.top();
			q.pop();
		} while (q.size() && vst[best_edge.v2]);
		if (vst[best_edge.v2])	// the queue is exhausted
			break;
		vst[best_edge.v2] = true;
		cnt ++;
		callback_edge(best_edge.v1, best_edge.v2);
		enqueue_edges_from(best_edge.v2);
	}
	if (cnt != n)
	{
		printf("Found a tree of size %d!=%d, images are not connected well!",cnt, n);
		exit(1);
	}
}

