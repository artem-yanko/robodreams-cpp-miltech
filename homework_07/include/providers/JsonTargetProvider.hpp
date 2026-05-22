#pragma once

  #include "interfaces/ITargetProvider.hpp"

  class JsonTargetProvider : public ITargetProvider {
  public:
      ~JsonTargetProvider() override;

      bool loadTargets(double arrayTimeStep) override;
      void setSimulationTime(double time) override;

      int getTargetCount() override;
      Target getTarget(int index) override;

  private:
      void clearTargets();

      TargetData targets{};
      double arrayTimeStep{};
      double simulationTime{};
  };