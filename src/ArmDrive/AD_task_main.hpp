#ifndef AD_TASK_MAIN_HPP_
#define AD_TASK_MAIN_HPP_

namespace ADT{

enum MSG_ID {
  REQ_MOVE_DIR = 0x01,
  MSG_UNKNOWN  = 0xFF,
};

void prepare_task();
void main(void* params);


};


#endif