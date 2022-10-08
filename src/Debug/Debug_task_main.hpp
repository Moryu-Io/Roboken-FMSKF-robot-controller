#ifndef DEBUG_TASK_MAIN_HPP_
#define DEBUG_TASK_MAIN_HPP_

namespace DEBUG{

void prepare_task();
void main(void* params);

extern char EXT_PRINT_BUF[1024];
void print(char* _buf, uint32_t _size);

void record_proc_load(uint8_t prc_id, uint8_t is_finish);


};

#endif