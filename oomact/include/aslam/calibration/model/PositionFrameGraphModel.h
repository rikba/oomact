#ifndef OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_FRAME_GRAPH_MODEL_H_
#define OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_FRAME_GRAPH_MODEL_H_
#include <aslam/calibration/model/Model.h>

namespace aslam {
namespace calibration {

class PositionFrameGraph;

class PositionFrameGraphModel : public Model {
 public:
  PositionFrameGraphModel(ValueStoreRef config, std::shared_ptr<ConfigPathResolver> configPathResolver = nullptr, const std::vector<const Frame *> frames = {});
  virtual ~PositionFrameGraphModel();

  ModelAtTime getAtTime(Timestamp timestamp, int maximalDerivativeOrder, const ModelSimplification & simplification) const override;
  ModelAtTime getAtTime(const BoundedTimeExpression & boundedTimeExpresion, int maximalDerivativeOrder, const ModelSimplification & simplification) const override;

  void init() override;

 protected:
  void registerModule(Module & m) override;
 private:
  template <typename Time> friend class PositionFrameGraphModelAtTimeImpl;
  std::unique_ptr<PositionFrameGraph> frameGraph_;
};

} /* namespace calibration */
} /* namespace aslam */

#endif /* OOMACT_ASLAM_CALIBRATION_MODEL_POSITION_FRAME_GRAPH_MODEL_H_ */
