#ifndef LOGGER_TASK_MAIN_HPP_
#define LOGGER_TASK_MAIN_HPP_

namespace LGT {

void prepare_task();
void main(void *params);

void push_buffer(char *_buf, uint32_t _size);

} // namespace LGT

#endif