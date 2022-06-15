#ifndef AD_TASK_MAIN_HPP_
#define AD_TASK_MAIN_HPP_

namespace ADT{

enum MSG_ID {
  REQ_MOVE_DIR = 0x01,
  MSG_UNKNOWN  = 0xFF,
};

enum MODE_ID { 
  OFF,
  INIT,
  POSITIONING,
  ERROR
};

void prepare_task();
void main(void* params);

void set_next_mode(MODE_ID _id);


};


#endif