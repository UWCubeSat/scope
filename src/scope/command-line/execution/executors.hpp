#ifndef SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
#define SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_

#include <memory>

#include "scope/command-line/parsing/options.hpp"

namespace scope {

class CalibrationAlgorithm {
 public:
    virtual ~CalibrationAlgorithm() = default;
    virtual void Calibrate(const CalibrationOptions &options) = 0;
};

class PipelineExecutor {
 public:
    virtual ~PipelineExecutor() = default;
    virtual void ExecutePipeline() = 0;
    virtual void OutputResults() = 0;
};

class CalibrationPipelineExecutor : public PipelineExecutor {
 public:
    explicit CalibrationPipelineExecutor(
        CalibrationOptions &&options,
        std::unique_ptr<CalibrationAlgorithm> calibrationAlgorithm);

    void ExecutePipeline() override;
    void OutputResults() override;

 private:
    const CalibrationOptions options_;
    std::unique_ptr<CalibrationAlgorithm> calibrationAlgorithm_;
    bool executed_ = false;
};

}  // namespace scope

#endif  // SRC_SCOPE_COMMAND_LINE_EXECUTION_EXECUTORS_HPP_
