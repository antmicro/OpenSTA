// OpenSTA, Static Timing Analyzer
// Copyright (c) 2024, Parallax Software, Inc.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include "ClkSkew.hh"

#include <cmath> // abs

#include "Report.hh"
#include "Debug.hh"
#include "DispatchQueue.hh"
#include "Fuzzy.hh"
#include "Units.hh"
#include "TimingArc.hh"
#include "Network.hh"
#include "Graph.hh"
#include "Sdc.hh"
#include "Bfs.hh"
#include "PathVertex.hh"
#include "StaState.hh"
#include "PathAnalysisPt.hh"
#include "SearchPred.hh"
#include "Search.hh"
#include "Crpr.hh"

namespace sta {

using std::abs;

// Source/target clock skew.
class ClkSkew
{
public:
  ClkSkew();
  ClkSkew(PathVertex *src_path,
	  PathVertex *tgt_path,
	  StaState *sta);
  ClkSkew(const ClkSkew &clk_skew);
  void operator=(const ClkSkew &clk_skew);
  PathVertex *srcPath() { return &src_path_; }
  PathVertex *tgtPath() { return &tgt_path_; }
  float srcLatency(StaState *sta);
  float tgtLatency(StaState *sta);
  Crpr crpr(StaState *sta);
  float skew() const { return skew_; }

private:
  PathVertex src_path_;
  PathVertex tgt_path_;
  float skew_;
};

ClkSkew::ClkSkew() :
  skew_(0.0)
{
}

ClkSkew::ClkSkew(PathVertex *src_path,
		 PathVertex *tgt_path,
		 StaState *sta)
{
  src_path_ = src_path;
  tgt_path_ = tgt_path;
  skew_ = srcLatency(sta) - tgtLatency(sta) - delayAsFloat(crpr(sta));
}

ClkSkew::ClkSkew(const ClkSkew &clk_skew)
{
  src_path_ = clk_skew.src_path_;
  tgt_path_ = clk_skew.tgt_path_;
  skew_ = clk_skew.skew_;
}

void
ClkSkew::operator=(const ClkSkew &clk_skew)
{
  src_path_ = clk_skew.src_path_;
  tgt_path_ = clk_skew.tgt_path_;
  skew_ = clk_skew.skew_;
}

float
ClkSkew::srcLatency(StaState *sta)
{
  Arrival src_arrival = src_path_.arrival(sta);
  return delayAsFloat(src_arrival) - src_path_.clkEdge(sta)->time();
}

float
ClkSkew::tgtLatency(StaState *sta)
{
  Arrival tgt_arrival = tgt_path_.arrival(sta);
  return delayAsFloat(tgt_arrival) - tgt_path_.clkEdge(sta)->time();
}

Crpr
ClkSkew::crpr(StaState *sta)
{
  CheckCrpr *check_crpr = sta->search()->checkCrpr();
  return check_crpr->checkCrpr(&src_path_, &tgt_path_);
}

////////////////////////////////////////////////////////////////

ClkSkews::ClkSkews(StaState *sta) :
  StaState(sta)
{
}

void
ClkSkews::reportClkSkew(ClockSet *clks,
			const Corner *corner,
			const SetupHold *setup_hold,
			int digits)
{
  ClkSkewMap skews;
  findClkSkew(clks, corner, setup_hold, skews);

  // Sort the clocks to report in a stable order.
  ClockSeq sorted_clks;
  for (Clock *clk : *clks)
    sorted_clks.push_back(clk);
  sort(sorted_clks, ClkNameLess());

  Unit *time_unit = units_->timeUnit();
  ClockSeq::Iterator clk_iter2(sorted_clks);
  while (clk_iter2.hasNext()) {
    Clock *clk = clk_iter2.next();
    report_->reportLine("Clock %s", clk->name());
    ClkSkew *clk_skew = skews.findKey(clk);
    if (clk_skew) {
      report_->reportLine("Latency      CRPR       Skew");
      PathVertex *src_path = clk_skew->srcPath();
      PathVertex *tgt_path = clk_skew->tgtPath();
      report_->reportLine("%s %s",
                          sdc_network_->pathName(src_path->pin(this)),
                          src_path->transition(this)->asString());
      report_->reportLine("%7s",
                          time_unit->asString(clk_skew->srcLatency(this), digits));
      report_->reportLine("%s %s",
                          sdc_network_->pathName(tgt_path->pin(this)),
                          tgt_path->transition(this)->asString());
      report_->reportLine("%7s   %7s    %7s",
                          time_unit->asString(clk_skew->tgtLatency(this), digits),
                          time_unit->asString(delayAsFloat(-clk_skew->crpr(this)),
                                              digits),
                          time_unit->asString(clk_skew->skew(), digits));
    }
    else
      report_->reportLine("No launch/capture paths found.");
    report_->reportBlankLine();
  }

  skews.deleteContents();
}

float
ClkSkews::findWorstClkSkew(const Corner *corner,
                           const SetupHold *setup_hold)
{
  ClockSet clks;
  for (Clock *clk : *sdc_->clocks())
    clks.insert(clk);
  ClkSkewMap skews;
  findClkSkew(&clks, corner, setup_hold, skews);
  float worst_skew = 0.0;
  for (auto clk_skew_itr : skews) {
    ClkSkew *clk_skew = clk_skew_itr.second;
    float skew = clk_skew->skew();
    if (abs(skew) > abs(worst_skew))
      worst_skew = skew;
  }
  skews.deleteContents();
  return worst_skew;
}

void
ClkSkews::findClkSkew(ClockSet *clks,
                      const Corner *corner,
                      const SetupHold *setup_hold,
                      ClkSkewMap &skews)
{
  const auto findClkSkew = [this, clks, corner, setup_hold](ClkSkewMap& skews, Vertex* src_vertex) {
    if (hasClkPaths(src_vertex, clks)) {
      VertexOutEdgeIterator edge_iter(src_vertex, graph_);
      while (edge_iter.hasNext()) {
        Edge *edge = edge_iter.next();
        if (edge->role()->genericRole() == TimingRole::regClkToQ()) {
          Vertex *q_vertex = edge->to(graph_);
          RiseFall *rf = edge->timingArcSet()->isRisingFallingEdge();
          RiseFallBoth *src_rf = rf
              ? rf->asRiseFallBoth()
              : RiseFallBoth::riseFall();
          findClkSkewFrom(src_vertex, q_vertex, src_rf, clks, corner, setup_hold, skews);
        }
      }
    }
  };
  if (dispatch_queue_) {
    std::vector<ClkSkewMap> partial_skews(thread_count_, skews);
    for (Vertex *src_vertex : *graph_->regClkVertices()) {
      dispatch_queue_->dispatch([=, &partial_skews](int i) {
        findClkSkew(partial_skews[i], src_vertex);
      });
    }
    dispatch_queue_->finishTasks();
    for (size_t i = 0; i < partial_skews.size(); i++) {
      for (auto [clk, partial_skew] : partial_skews[i]) {
        ClkSkew *&final_skew = skews[clk];
        if (!final_skew) {
          final_skew = partial_skew;
        }
        else if (abs(partial_skew->skew()) > abs(final_skew->skew())) {
          delete final_skew;
          final_skew = partial_skew;
        }
        else {
          delete partial_skew;
        }
      }
    }
  }
  else {
    for (Vertex *src_vertex : *graph_->regClkVertices()) {
      findClkSkew(skews, src_vertex);
    }
  }
}

bool
ClkSkews::hasClkPaths(Vertex *vertex,
		      ClockSet *clks)
{
  VertexPathIterator path_iter(vertex, this);
  while (path_iter.hasNext()) {
    PathVertex *path = path_iter.next();
    const Clock *path_clk = path->clock(this);
    if (clks->hasKey(const_cast<Clock*>(path_clk)))
      return true;
  }
  return false;
}

void
ClkSkews::findClkSkewFrom(Vertex *src_vertex,
			  Vertex *q_vertex,
			  RiseFallBoth *src_rf,
			  ClockSet *clks,
			  const Corner *corner,
			  const SetupHold *setup_hold,
			  ClkSkewMap &skews)
{
  VertexSet endpoints = findFanout(q_vertex);
  VertexSet::Iterator end_iter(endpoints);
  while (end_iter.hasNext()) {
    Vertex *end = end_iter.next();
    VertexInEdgeIterator edge_iter(end, graph_);
    while (edge_iter.hasNext()) {
      Edge *edge = edge_iter.next();
      TimingRole *role = edge->role();
      if (role->isTimingCheck()
	  && ((setup_hold == SetupHold::max()
	       && role->genericRole() == TimingRole::setup())
	      || ((setup_hold == SetupHold::min()
		   && role->genericRole() == TimingRole::hold())))) {
	Vertex *tgt_vertex = edge->from(graph_);
	RiseFall *tgt_rf1 = edge->timingArcSet()->isRisingFallingEdge();
	RiseFallBoth *tgt_rf = tgt_rf1
	  ? tgt_rf1->asRiseFallBoth()
	  : RiseFallBoth::riseFall();
	findClkSkew(src_vertex, src_rf, tgt_vertex, tgt_rf,
		    clks, corner, setup_hold, skews);
      }
    }
  }
}

void
ClkSkews::findClkSkew(Vertex *src_vertex,
		      RiseFallBoth *src_rf,
		      Vertex *tgt_vertex,
		      RiseFallBoth *tgt_rf,
		      ClockSet *clks,
		      const Corner *corner,
		      const SetupHold *setup_hold,
		      ClkSkewMap &skews)
{
  Unit *time_unit = units_->timeUnit();
  const SetupHold *tgt_min_max = setup_hold->opposite();
  VertexPathIterator src_iter(src_vertex, this);
  while (src_iter.hasNext()) {
    PathVertex *src_path = src_iter.next();
    const Clock *src_clk = src_path->clock(this);
    if (src_rf->matches(src_path->transition(this))
	&& src_path->minMax(this) == setup_hold
	&& clks->hasKey(const_cast<Clock*>(src_clk))) {
      Corner *src_corner = src_path->pathAnalysisPt(this)->corner();
      if (corner == nullptr
	  || src_corner == corner) {
	VertexPathIterator tgt_iter(tgt_vertex, this);
	while (tgt_iter.hasNext()) {
	  PathVertex *tgt_path = tgt_iter.next();
	  const Clock *tgt_clk = tgt_path->clock(this);
	  if (tgt_clk == src_clk
	      && tgt_path->isClock(this)
	      && tgt_rf->matches(tgt_path->transition(this))
	      && tgt_path->minMax(this) == tgt_min_max
	      && tgt_path->pathAnalysisPt(this)->corner() == src_corner) {
	    ClkSkew probe(src_path, tgt_path, this);
	    ClkSkew *clk_skew = skews.findKey(const_cast<Clock*>(src_clk));
	    debugPrint(debug_, "clk_skew", 2,
                       "%s %s %s -> %s %s %s crpr = %s skew = %s",
                       network_->pathName(src_path->pin(this)),
                       src_path->transition(this)->asString(),
                       time_unit->asString(probe.srcLatency(this)),
                       network_->pathName(tgt_path->pin(this)),
                       tgt_path->transition(this)->asString(),
                       time_unit->asString(probe.tgtLatency(this)),
                       delayAsString(probe.crpr(this), this),
                       time_unit->asString(probe.skew()));
	    if (clk_skew == nullptr) {
	      clk_skew = new ClkSkew(probe);
	      skews[src_clk] = clk_skew;
	    }
	    else if (abs(probe.skew()) > abs(clk_skew->skew()))
	      *clk_skew = probe;
	  }
	}
      }
    }
  }
}

class FanOutSrchPred : public SearchPred1
{
public:
  FanOutSrchPred(const StaState *sta);
  virtual bool searchThru(Edge *edge);
};

FanOutSrchPred::FanOutSrchPred(const StaState *sta) :
  SearchPred1(sta)
{
}

bool
FanOutSrchPred::searchThru(Edge *edge)
{
  TimingRole *role = edge->role();
  return SearchPred1::searchThru(edge)
    && (role == TimingRole::wire()
        || role == TimingRole::combinational()
        || role == TimingRole::tristateEnable()
        || role == TimingRole::tristateDisable());
}

VertexSet
ClkSkews::findFanout(Vertex *from)
{
  debugPrint(debug_, "fanout", 1, "%s",
             from->name(sdc_network_));
  VertexSet endpoints(graph_);
  FanOutSrchPred pred(this);
  BfsFwdIterator fanout_iter(BfsIndex::other, &pred, this);
  fanout_iter.enqueue(from);
  while (fanout_iter.hasNext()) {
    Vertex *fanout = fanout_iter.next();
    if (fanout->hasChecks()) {
      debugPrint(debug_, "fanout", 1, " endpoint %s",
                 fanout->name(sdc_network_));
      endpoints.insert(fanout);
    }
    fanout_iter.enqueueAdjacentVertices(fanout);
  }
  return endpoints;
}

////////////////////////////////////////////////////////////////

void
ClkSkews::findClkDelays(const Clock *clk,
                        // Return values.
                        ClkDelays &delays)
{
  for (Vertex *clk_vertex : *graph_->regClkVertices()) {
    VertexPathIterator path_iter(clk_vertex, this);
    while (path_iter.hasNext()) {
      PathVertex *path = path_iter.next();
      const ClockEdge *path_clk_edge = path->clkEdge(this);
      if (path_clk_edge) {
        const RiseFall *clk_rf = path_clk_edge->transition();
        const Clock *path_clk = path_clk_edge->clock();
        if (path_clk == clk) {
          Arrival arrival = path->arrival(this);
          Delay clk_delay = delayAsFloat(arrival) - path_clk_edge->time();
          const MinMax *min_max = path->minMax(this);
          const RiseFall *rf = path->transition(this);
          delays[clk_rf->index()][rf->index()].setValue(min_max, clk_delay);
        }
      }
    }
  }
}

} // namespace
