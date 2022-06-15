#ifndef AD_MODE_BASE_HPP_
#define AD_MODE_BASE_HPP_

#include "global_config.hpp"
#include "AD_joint_base.hpp"

namespace ADT {

/**
 * @brief ArmDriveTask Mode基底クラス
 * 
 */
class ADTModeBase {
public:
  ADTModeBase(){};

  void init(){
    is_comp = false;
    doInit();
  };
  virtual void doInit() = 0;
  virtual void update() = 0;
  virtual void end()    = 0;

  virtual bool isCompleted() { return is_comp; }

  static JointBase* P_JOINT_[JointAxis::J_NUM];
  static float FL_CYCLE_TIME_S;

protected:
  bool is_comp;
};

/**
 * @brief Mode トルク完全OFFクラス
 * 
 */
class ADTModeOff : public ADTModeBase {
public:
  void doInit() override {};
  void end() override {};
  void update() override {
    for(int i=0; i<JointAxis::J_NUM; i++){
        ADTModeBase::P_JOINT_[i]->set_torque_on(false);
    }

    is_comp = true;
  };

};


}; // namespace ADT

#endif