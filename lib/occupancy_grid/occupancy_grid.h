#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H
#include <stdint.h>

typedef struct {
    uint8_t width;
    uint8_t height;
    uint8_t *cells; // dynamically allocated [width*height]
} Grid_t;

void grid_init(Grid_t *grid, uint8_t width, uint8_t height);
void grid_update_cell(Grid_t *grid, int x, int y, uint8_t occupied);
uint8_t grid_get_cell(Grid_t *grid, int x, int y);

#endif
