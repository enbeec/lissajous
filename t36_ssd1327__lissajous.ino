/* -------- MAIN DISPLAY ---------------------------------------------------- */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include "Font5x7FixedMono.h"

#define OLED_CLK 13
#define OLED_MOSI 11
#define OLED_CS 10
#define OLED_DC 8
#define OLED_RESET -1

Adafruit_SSD1327 display(128, 128, &SPI, OLED_DC, OLED_RESET, OLED_CS);

void display_setup(void) {
  Serial.println("-> connecting display");
  if ( ! display.begin(0x3D) ) {
    Serial.println("Unable to initialize OLED");
    while (1) yield();
  }

  display.setTextSize(1);
  display.setTextWrap(false);
  display.setTextColor(SSD1327_WHITE);
  display.setFont(&Font5x7FixedMono);
  display.setRotation(1);
  display.clearDisplay();
  display.display();
}

// -------- INPUT ----------------------------------------------------------- */
#include <Bounce.h>
#define BOUNCE_MS 10

#define NUM_BUTTONS 6
#define B0 0
#define B1 1
#define B2 2
#define B3 3
#define B4 4
#define B5 5

// TODO rewrite this as a class
namespace buttons {
void setup(void) {
  pinMode(B0, INPUT_PULLUP);
  pinMode(B1, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);
  pinMode(B3, INPUT_PULLUP);
  pinMode(B4, INPUT_PULLUP);
  pinMode(B5, INPUT_PULLUP);
};

Bounce debounced[NUM_BUTTONS] = {
  Bounce(B0, BOUNCE_MS),
  Bounce(B1, BOUNCE_MS),
  Bounce(B2, BOUNCE_MS),
  Bounce(B3, BOUNCE_MS),
  Bounce(B4, BOUNCE_MS),
  Bounce(B5, BOUNCE_MS)
};

bool held[NUM_BUTTONS];
bool down[NUM_BUTTONS];
bool up[NUM_BUTTONS];

void update(void) {
  /*     REMINDER:
    - myButton.fallingEdge() == ON
    - myButton.risingEdge() == OFF
    ...because our buttons are wired active LOW
  */
  for (int i = 0; i < NUM_BUTTONS; i++) {
    down[i] = false;
    up[i] = false;
    buttons::debounced[i].update();
    if (buttons::debounced[i].fallingEdge()) {
      buttons::down[i] = true;
      buttons::held[i] = true;
    } else if (buttons::debounced[i].risingEdge()) {
      buttons::up[i] = true;
      buttons::held[i] = false;
    }
  }
}
};

/* ------- GRID COORDS ------------------------------------------------------ */
#define CIRCLE_RADIUS 10
#define cols 4
#define rows cols
// array of center coordinates
int grid[4][4][2] {
  {{22, 20}, {48, 20}, {74, 20}, {100, 20}},
  {{22, 46}, {48, 46}, {74, 46}, {100, 46}},
  {{22, 72}, {48, 72}, {74, 72}, {100, 72}},
  {{22, 98}, {48, 98}, {74, 98}, {100, 98}}
};

/* ------- TRIG ------------------------------------------------------------- */
double theta = 0;
double polar_xs[rows] = { 0, 0, 0 };
double polar_ys[cols] = { 1, 1, 1 };

#define DELTA_MAX 0.2
#define DELTA_MIN 0.01
#define DELTA_STEP 0.01
float delta = 0.08;

void inc_delta(void) {
  if (delta + DELTA_STEP < DELTA_MAX) delta += DELTA_STEP;
}

void dec_delta(void) {
  if (delta - DELTA_STEP > DELTA_MIN) delta -= DELTA_STEP;
}

/* ------- PIXELS ----------------------------------------------------------- */
int pixels[cols][rows][3];
void clear_pixels(void) {
  for (int j = 0; j < cols; j++) {
    for (int k = 0; k < rows; k++) {
      pixels[j][k][0] = 0;
      pixels[j][k][1] = 0;
      pixels[j][k][2] = 0;
    }
  }

}

void add_pixel(int col, int row, uint8_t x, uint8_t y, uint8_t z) {
  pixels[col][row][0] = x;
  pixels[col][row][1] = y;
  pixels[col][row][2] = z;
}



#define pixel_A pixels[0][0][0], pixels[0][0][1]
#define pixel_B pixels[0][1][0], pixels[0][1][1]
#define pixel_C pixels[0][2][0], pixels[0][2][1]
#define pixel_D pixels[0][3][0], pixels[0][3][1]
#define pixel_E pixels[1][0][0], pixels[1][0][1]
#define pixel_F pixels[1][1][0], pixels[1][1][1]
#define pixel_G pixels[1][2][0], pixels[1][2][1]
#define pixel_H pixels[1][3][0], pixels[1][3][1]
#define pixel_I pixels[2][0][0], pixels[2][0][1]
#define pixel_J pixels[2][1][0], pixels[2][1][1]
#define pixel_K pixels[2][2][0], pixels[2][2][1]
#define pixel_L pixels[2][3][0], pixels[2][3][1]
#define pixel_M pixels[3][0][0], pixels[3][0][1]
#define pixel_N pixels[3][1][0], pixels[3][1][1]
#define pixel_O pixels[3][2][0], pixels[3][2][1]
#define pixel_P pixels[3][3][0], pixels[3][3][1]

/* ------- TRIANGLE OCTOGON ------------------------------------------------- */
#define OCTOGON_TRIANGLES 14
int tri_oct_brightness_min = 2;
int tri_oct_brightness_max = 14;
int tri_oct_brightness[OCTOGON_TRIANGLES] = {
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max)),
  int(random(tri_oct_brightness_min, tri_oct_brightness_max))
};

int t_o_b(int index) {
  return tri_oct_brightness[index];
}

/* ------- SCALES ----------------------------------------------------------- */
enum SCALES { MULT, PYTHAG, JUST, NUM_SCALES };
double scales[NUM_SCALES][cols][2] = {
  // MULT
  { {1, 1}, {2, 2}, {3, 3}, {4, 4} },
  // PYTHAG
  { {1, 1}, {9, 8}, {4, 3}, {3, 2} },
  // JUST
  { {1, 1}, {9, 8}, {6, 5}, {5, 4} },
};

int current_scale = JUST;
void next_scale(void) {
  current_scale++;
  if (current_scale == NUM_SCALES) current_scale = MULT;
}

/* -------------------------------------------------------------------------- */
uint8_t tick = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");

  // CONNECT BUTTONS
  buttons::setup();

  // CONNECT MAIN DISPLAY
  display_setup();
}

void loop() {
  tick++;
  bool every_other = tick % 2 == 0;
  bool every_third = tick % 3 == 0;
  bool every_sixteenth = tick % 16 == 0;
  bool every_eighth = tick % 8  == 0;

  buttons::update();

  /* BUTTON LAYOUT
                 4
      0   1   2     5
                 3
  */

#ifdef DEBUG
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttons::held[i]) Serial.printf("%i is held\n", i);
    if (buttons::down[i]) Serial.printf("%i down\n", i);
    if (buttons::up[i]) Serial.printf("%i up\n", i);
  }
#endif


  if (buttons::down[2]) {
    clear_pixels();
    next_scale();
  }

  if (buttons::down[0] || (every_eighth && buttons::held[0])) dec_delta();
  if (buttons::down[1] || (every_eighth && buttons::held[1])) inc_delta();

  theta += ( delta / 2 );

  // UPDATE POLAR OFFSETS
  for (int i = 0; i < cols; i++) {
    if (current_scale != JUST) {
      polar_xs[i] = sin(theta * (scales[current_scale][i][0])) * CIRCLE_RADIUS;
    } else {
      polar_xs[i] = cos(theta * (scales[current_scale][i][0])) * CIRCLE_RADIUS;
    }
    polar_ys[i] = sin(theta * (scales[current_scale][i][1])) * CIRCLE_RADIUS;
  }

  // UPDATE PIXELS
  for (int j = 0; j < cols; j++) {
    for (int k = 0; k < rows; k++) {
      add_pixel(
        j, k,
        grid[j][k][0] + int(polar_xs[j]),
        grid[j][k][1] + int(polar_ys[k]),
        60
      );
    }
  }

  // REDRAW
  display.clearDisplay();

  /*  ___CELL MACROS___

        A   B   C   D

        E   F   G   H

        I   J   K   L

        M   N   O   P
  */

#ifdef DRAW_MESH
  // draw triangle mesh
  // top left box
  display.drawTriangle(pixel_A, pixel_B, pixel_E, 8);
  display.drawTriangle(pixel_B, pixel_F, pixel_E, 8);
  // top middle box
  display.drawTriangle(pixel_B, pixel_C, pixel_F, 8);
  display.drawTriangle(pixel_C, pixel_G, pixel_F, 8);
  // top right box **FLIPPED**
  display.drawTriangle(pixel_C, pixel_H, pixel_G, 8);
  display.drawTriangle(pixel_C, pixel_D, pixel_H, 8);
  // mid left box
  display.drawTriangle(pixel_E, pixel_F, pixel_I, 8);
  display.drawTriangle(pixel_F, pixel_J, pixel_I, 8);
  // mid middle box
  display.drawTriangle(pixel_F, pixel_G, pixel_J, 8);
  display.drawTriangle(pixel_G, pixel_K, pixel_J, 8);
  // mid right box
  display.drawTriangle(pixel_G, pixel_H, pixel_K, 8);
  display.drawTriangle(pixel_H, pixel_L, pixel_K, 8);
  // bot left box **FLIPPED**
  display.drawTriangle(pixel_I, pixel_N, pixel_M, 8);
  display.drawTriangle(pixel_I, pixel_J, pixel_N, 8);
  // bot middle box
  display.drawTriangle(pixel_J, pixel_K, pixel_N, 8);
  display.drawTriangle(pixel_K, pixel_O, pixel_N, 8);
  // bot right box
  display.drawTriangle(pixel_K, pixel_L, pixel_O, 8);
  display.drawTriangle(pixel_L, pixel_P, pixel_O, 8);
#endif

  // fill the octogon (14 triangles)
  display.fillTriangle(pixel_B, pixel_F, pixel_E, t_o_b(0));
  display.fillTriangle(pixel_B, pixel_C, pixel_F, t_o_b(1));
  display.fillTriangle(pixel_C, pixel_G, pixel_F, t_o_b(2));
  display.fillTriangle(pixel_C, pixel_H, pixel_G, t_o_b(3));
  display.fillTriangle(pixel_E, pixel_F, pixel_I, t_o_b(4));
  display.fillTriangle(pixel_F, pixel_J, pixel_I, t_o_b(5));
  display.fillTriangle(pixel_F, pixel_G, pixel_J, t_o_b(6));
  display.fillTriangle(pixel_G, pixel_K, pixel_J, t_o_b(7));
  display.fillTriangle(pixel_G, pixel_H, pixel_K, t_o_b(8));
  display.fillTriangle(pixel_H, pixel_L, pixel_K, t_o_b(9));
  display.fillTriangle(pixel_I, pixel_J, pixel_N, t_o_b(10));
  display.fillTriangle(pixel_J, pixel_K, pixel_N, t_o_b(11));
  display.fillTriangle(pixel_K, pixel_O, pixel_N, t_o_b(12));
  display.fillTriangle(pixel_K, pixel_L, pixel_O, t_o_b(13));

  // TODO better timing model
  delay(10);
  display.display();

}
