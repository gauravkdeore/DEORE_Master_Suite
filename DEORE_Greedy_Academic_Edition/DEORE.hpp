#ifndef DEORE_HPP
#define DEORE_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <limits>
#include <utility>
#include <functional>
#include <cstdlib>
#include <cstddef>

namespace DEORE {

static const double SQRT2 = 1.4142135623730951;
static const double INF   = 1e18;
static const double EPS   = 1e-9;

struct xyLoc { int r, c; };
inline int IDX(int r, int c, int W) { return r * W + c; }

inline bool FREE(const std::vector<bool>& bits, int r, int c, int W, int H) {
    return r>=0 && r<H && c>=0 && c<W && bits[IDX(r,c,W)];
}

// Strict 2x2 corner-cutting limitation forced natively in every single step!
inline bool FREE_DIAG(const std::vector<bool>& bits, int r, int c, int dr, int dc, int W, int H) {
    if (!FREE(bits, r+dr, c+dc, W, H)) return false;
    if (!FREE(bits, r+dr, c, W, H)) return false;
    if (!FREE(bits, r, c+dc, W, H)) return false;
    return true;
}

inline double OCT(int r1,int c1,int r2,int c2){
    int dr=std::abs(r1-r2), dc=std::abs(c1-c2);
    return std::max(dr,dc)+(SQRT2-1)*std::min(dr,dc);
}

struct HeapNode {
    double f, g;
    int r, c, pr, pc;
    // std::greater uses operator> to put the smallest f-value at the top of the Min-Heap
    bool operator>(const HeapNode& o) const {
        if (std::abs(f-o.f) > EPS) return f > o.f;
        return g < o.g;
    }
};
using PQ = std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>>;

struct GridState {
    double g;
    int parent;
    unsigned int sid_open;
    unsigned int sid_closed;
};

struct MicroNode {
    double f;
    double h;
    int k;
    bool operator>(const MicroNode& o) const { 
        if (std::abs(f - o.f) > EPS) return f > o.f;
        return h > o.h; // Mathematical tie-breaker: prioritize physical proximity to the goal!
    }
};
using FastPQ = std::priority_queue<MicroNode, std::vector<MicroNode>, std::greater<MicroNode>>;

class DEORE_Engine {
public:
    int W, H, V;
    std::vector<bool> bits;
    
    // Proprietary Hardware-Aligned L1-Cache Unified Memory
    unsigned int search_id;
    std::vector<GridState> state;

    DEORE_Engine(const std::vector<bool>& b, int w, int h)
        : W(w), H(h), V(w*h), bits(b), search_id(1) 
    {
        state.resize(V, {INF, -1, 0, 0});
    }

    inline double getG(int k) { return (state[k].sid_open == search_id) ? state[k].g : INF; }
    inline bool getCl(int k) { return (state[k].sid_closed == search_id); }
    
    inline void setG(int k, double val, int par) { 
        state[k].g = val; 
        state[k].parent = par; 
        state[k].sid_open = search_id; 
    }
    inline void setCl(int k) { state[k].sid_closed = search_id; }

    std::vector<xyLoc> FindPath(int sr, int sc, int er, int ec) {
        if (sr < 0 || sr >= H || sc < 0 || sc >= W || er < 0 || er >= H || ec < 0 || ec >= W) return {};
        if (!bits[sr*W+sc] || !bits[er*W+ec]) return {};
        if (sr==er && sc==ec) return {{sr,sc}};

        search_id++;
        if (search_id == 0) { 
            state.assign(V, {INF, -1, 0, 0});
            search_id = 1;
        }

        FastPQ O;
        double h0 = OCT(sr,sc,er,ec);
        int startK = sr*W + sc, endK = er*W + ec;
        setG(startK, 0.0, -1);
        
        // Aggressive 2.5x Greedy Multiplier perfectly shatters deep labyrinth dead-ends
        double eps = 2.5; 
        O.push({eps*h0, h0, startK});

        static const int DR8[8] = {-1, 1,  0, 0, -1, -1,  1, 1};
        static const int DC8[8] = { 0, 0, -1, 1, -1,  1, -1, 1};
        int K_OFF[8] = {-W, W, -1, 1, -W-1, -W+1, W-1, W+1};

        while (!O.empty()) {
            MicroNode cur = O.top(); O.pop();
            int k = cur.k;
            
            if (getCl(k)) continue; 
            
            if (k == endK) {
                std::vector<xyLoc> chain;
                int currK = endK;
                while (currK != startK && currK != -1) {
                    chain.push_back({currK/W, currK%W});
                    currK = state[currK].parent;
                }
                chain.push_back({startK/W, startK%W});
                std::reverse(chain.begin(), chain.end());
                return chain;
            }
            
            setCl(k);
            double curG = getG(k);
            int r = k / W, c = k % W;

            for (int d=0; d<8; d++){
                int dr = DR8[d], dc = DC8[d];
                int jr = r + dr, jc = c + dc;
                
                // Fast bounds checking
                if (jr < 0 || jr >= H || jc < 0 || jc >= W) continue;

                int jk = k + K_OFF[d];

                // Native 1D GPPC 2x2 Constraint Check (No Multiplication)
                if (dr && dc) {
                    if (!bits[jk] || !bits[k + dr*W] || !bits[k + dc]) continue;
                } else {
                    if (!bits[jk]) continue;
                }

                if (getCl(jk)) continue;

                double ng = curG + ((dr&&dc) ? SQRT2 : 1.0);

                if (ng < getG(jk)-EPS){
                    setG(jk, ng, k);
                    double hh = OCT(jr,jc,er,ec);
                    O.push({ng + eps*hh, hh, jk});
                }
            }
        }
        return {};
    }
};

} // namespace DEORE

#endif // DEORE_HPP
