#include <datatypes.h>

// void readEngineerMcuData(McuData *mcudata, uint8_t *start, uint8_t *state,
//                          uint8_t *view, uint8_t *barrier_ok) {
//     *start = mcudata->start;
//     *state = mcudata->type;
//     *view = mcudata->view;
//     *barrier_ok = mcudata->barrier_ok;
// }

void readEngineerMcuData(McuData *mcudata, uint8_t *view, uint8_t *state) {
    *view = mcudata->view;
    *state = mcudata->state;
}