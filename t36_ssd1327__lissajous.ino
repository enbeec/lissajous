/* -------- DISPLAY --------------------------------------------------------- */
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

#define NUM_BUTTONS 3
#define B0 0
#define B1 1
#define B2 2

void buttons_setup(void) {
  Serial.println("-> connecting buttons");
  pinMode(B0, INPUT_PULLUP);
  pinMode(B1, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);
}

Bounce buttons[NUM_BUTTONS] = {
  Bounce(B0, BOUNCE_MS),
  Bounce(B1, BOUNCE_MS),
  Bounce(B2, BOUNCE_MS)
};

bool held[NUM_BUTTONS] = { false, false, false };

bool buttons_update(void) {
  bool result0 = buttons[0].update();
  bool result1 = buttons[1].update();
  bool result2 = buttons[2].update();
  return (result0 && result1 && result2);
}

/*
   REMINDER:
    - myButton.fallingEdge() == ON
    - myButton.risingEdge() == OFF
    ...because our buttons are wired active LOW
*/

void handle_inputs(void) {
  buttons_update();
  if (buttons[0].fallingEdge()) {
    held[0] = true;
  } else if (buttons[0].risingEdge()) {
    held[0] = false;
  }

  if (buttons[1].fallingEdge()) {
    held[1] = true;
  } else if (buttons[1].risingEdge()) {
    held[1] = false;
  }

  if (buttons[2].fallingEdge()) {
    held[2] = true;
  } else if (buttons[2].risingEdge()) {
    held[2] = false;
  }
}

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
#define DELTA_MIN 0.02
#define DELTA_STEP 0.01
float delta = 0.083;

void inc_delta(void) {
  if (delta + DELTA_STEP < DELTA_MAX) delta += DELTA_STEP;
}

void dec_delta(void) {
  if (delta - DELTA_STEP > DELTA_MIN) delta -= DELTA_STEP;
}

/* ------- PIXEL BUFFERS ---------------------------------------------------- */
#define PIXEL_NUM 96

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

/*  ___CELL MACROS___

      A   B   C   D

      E   F   G   H

      I   J   K   L

      M   N   O   P
*/

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

int current_scale = MULT;
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
  buttons_setup();

  // CONNECT DISPLAY
  display_setup();

  // LOCAL
}

void loop() {
  tick++;
  //bool every_other = tick % 2 == 0;
  //bool every_third = tick % 3 == 0;
  bool every_fourth  = tick % 4  == 0;

  handle_inputs();

  if (buttons[2].risingEdge() && (!held[0] && !held[1])) {
    //clear_pixels();
    next_scale();
  }

  if (buttons[0].fallingEdge() || (every_fourth && held[0])) dec_delta();
  if (buttons[1].fallingEdge() || (every_fourth && held[1])) inc_delta();

  theta += delta;

  // REDRAW
  display.clearDisplay();
  display.setTextColor(int(12 * cos(theta / 4)));

  for (int i = 0; i < cols; i++) {
    polar_xs[i] = sin(theta * (scales[current_scale][i][0])) * CIRCLE_RADIUS;
    polar_ys[i] = sin(theta * (scales[current_scale][i][1])) * CIRCLE_RADIUS;
  }

  for (int j = 0; j < cols; j++) {
    for (int k = 0; k < rows; k++) {
      add_pixel(
        j, k,
        grid[j][k][0] + int(polar_xs[j]),
        grid[j][k][1] + int(polar_ys[k]),
        60
      );

      display.drawCircle(
        grid[j][k][0] + int(polar_xs[j]),
        grid[j][k][1] + int(polar_ys[k]),
        2, SSD1327_WHITE
      );

      display.drawPixel(
        pixels[j][k][0],
        pixels[j][k][1],
        pixels[j][k][2] / 4
      );
    }
  }
  
  // DRAW LINES
  // using named macros for each current pixel

  // row ABCD
  display.drawLine(pixel_A, pixel_B, 8);
  display.drawLine(pixel_B, pixel_C, 8);
  display.drawLine(pixel_C, pixel_D, 8);

  // row EFGH
  display.drawLine(pixel_E, pixel_F, 8);
  display.drawLine(pixel_F, pixel_G, 8);
  display.drawLine(pixel_G, pixel_H, 8);

  // row IJKL
  display.drawLine(pixel_I, pixel_J, 8);
  display.drawLine(pixel_J, pixel_K, 8);
  display.drawLine(pixel_K, pixel_L, 8);

  // row MNOP
  display.drawLine(pixel_M, pixel_N, 8);
  display.drawLine(pixel_N, pixel_O, 8);
  display.drawLine(pixel_O, pixel_P, 8);

  // col AEIM
  display.drawLine(pixel_A, pixel_E, 8);
  display.drawLine(pixel_E, pixel_I, 8);
  display.drawLine(pixel_I, pixel_M, 8);

  // col BFJN
  display.drawLine(pixel_B, pixel_F, 8);
  display.drawLine(pixel_F, pixel_J, 8);
  display.drawLine(pixel_J, pixel_N, 8);

  // col CGKO
  display.drawLine(pixel_C, pixel_G, 8);
  display.drawLine(pixel_G, pixel_K, 8);
  display.drawLine(pixel_K, pixel_O, 8);

  // col DHLP
  display.drawLine(pixel_D, pixel_H, 8);
  display.drawLine(pixel_H, pixel_L, 8);
  display.drawLine(pixel_L, pixel_P, 8);

  // TODO better timing model
  delay(40);
  display.display();

}
