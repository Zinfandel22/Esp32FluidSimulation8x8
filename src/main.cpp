#include "config.h"
#include "grid.h"
#include "particle.h"
#include "utils.h"
#include "button.h"

#include <WiFi.h>
#include <esp_bt.h>
#include <esp_wifi.h>

#include "esp_sleep.h"

// global instances
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);
Grid grid;                         // single grid instance for the simulation
Particle particles[NUM_PARTICLES]; // array of all particles
ButtonHandler button;              // button

// timing control for fixed timestep
unsigned long lastStepTime = 0;

// gravity vector (updated from accelerometer each frame)
float gravityX = 0.0f;
float gravityY = -GRAVITY_MAGNITUDE; // default: pointing down
float currentAccelZ = 0.0f;          // ADD THIS: To track face-down state

// particle radius - calculated from cell size like reference code
float particleRadius = 0.0f;

// LED intensity buffers. Particles are splatted into the 8x8 display with
// bilinear weights, then lightly decayed to hide 8x8 quantization.
float ledIntensity[NUM_LEDS] = {0.0f};
float ledNextIntensity[NUM_LEDS] = {0.0f};

// --- Profiling Variables (Time in milliseconds) ---
float t_integrate = 0; // Movement & Gravity
float t_push = 0;      // Particle Separation (Heavy)
float t_collision = 0; // Wall handling
float t_grid_prep = 0; // Resetting/Clearing grid
float t_p2g = 0;       // Particle -> Grid Transfer
float t_density = 0;   // Density calculation
float t_solve = 0;     // Incompressibility (Heavy)
float t_g2p = 0;       // Grid -> Particle Transfer
float t_vis = 0;       // LED Visualization

void enterSleepMode()
{
  Serial.println("Entering low power sleep mode...");

  // Turn off the LEDs
  matrix.clear();
  matrix.show();

  // Configure the button to wake the ESP32 (Replace BUTTON_PIN if named differently in config.h)
  gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Pause the CPU and enter Light Sleep
  esp_light_sleep_start();

  // --- SYSTEM WAKES UP HERE ---
  Serial.println("Waking up!");

  // Reset the simulation timer so the physics engine resumes smoothly
  lastStepTime = millis();
}

// function to initialize particles in a block formation
void initializeParticles()
{
  float cell_size_x = PHYSICAL_WIDTH / GRID_SIZE_X;
  float cell_size_y = PHYSICAL_HEIGHT / GRID_SIZE_Y;
  float cell_size = (cell_size_x > cell_size_y) ? cell_size_x : cell_size_y;

  particleRadius = 0.3f * cell_size;

  float dx = 1.8f * particleRadius;
  float dy = sqrtf(3.0f) / 2.0f * dx;

  float available_width = PHYSICAL_WIDTH - 2.0f * cell_size - 2.0f * particleRadius;
  float available_height = PHYSICAL_HEIGHT - 2.0f * cell_size - 2.0f * particleRadius;

  int particles_per_row = (int)(available_width / dx);
  int particles_per_col = (int)(available_height / dy);

  while (particles_per_row * particles_per_col > NUM_PARTICLES)
  {
    if (particles_per_row > particles_per_col)
    {
      particles_per_row--;
    }
    else
    {
      particles_per_col--;
    }
  }

  float start_x = cell_size + particleRadius;
  float start_y = cell_size + particleRadius;

  int particle_index = 0;
  for (int j = 0; j < particles_per_col && particle_index < NUM_PARTICLES; j++)
  {
    for (int i = 0; i < particles_per_row && particle_index < NUM_PARTICLES; i++)
    {
      float offset = (j % 2 == 0) ? 0.0f : particleRadius;

      particles[particle_index].x_pos = (start_x + i * dx + offset);
      particles[particle_index].y_pos = (start_y + j * dy);
      particles[particle_index].vx = (0.0f);
      particles[particle_index].vy = (0.0f);
      particle_index++;
    }
  }

  while (particle_index < NUM_PARTICLES)
  {
    particles[particle_index].x_pos = (start_x);
    particles[particle_index].y_pos = (start_y);
    particles[particle_index].vx = (0.0f);
    particles[particle_index].vy = (0.0f);
    particle_index++;
  }

  Serial.print("Particle radius: ");
  Serial.print(particleRadius * 1000.0f, 3);
  Serial.println(" mm");
  Serial.print("Particles arranged: ");
  Serial.print(particles_per_row);
  Serial.print(" x ");
  Serial.println(particles_per_col);
}

// function to map particles to led matrix and display with dynamic vector-product gradient
void visualizeParticles()
{
  const float cell_size_x = PHYSICAL_WIDTH / GRID_SIZE_X;
  const float cell_size_y = PHYSICAL_HEIGHT / GRID_SIZE_Y;
  const float render_min_x = cell_size_x + particleRadius;
  const float render_min_y = cell_size_y + particleRadius;
  const float render_max_x = PHYSICAL_WIDTH - cell_size_x - particleRadius;
  const float render_max_y = PHYSICAL_HEIGHT - cell_size_y - particleRadius;

  // 8x8 scales map coordinates explicitly from 0.0f to 7.0f
  const float led_scale_x = 7.0f / (render_max_x - render_min_x);
  const float led_scale_y = 7.0f / (render_max_y - render_min_y);

  for (int i = 0; i < NUM_LEDS; i++)
  {
    ledNextIntensity[i] = 0.0f;
  }

  // Bilinear splatting into 8x8 matrix grid
  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    float fx = (particles[i].x_pos - render_min_x) * led_scale_x;
    float fy = (particles[i].y_pos - render_min_y) * led_scale_y;

    fx = utils::clamp(fx, 0.0f, 7.0f);
    fy = utils::clamp(fy, 0.0f, 7.0f);

    int x0 = (int)fx;
    int y0 = (int)fy;
    int x1 = (x0 < 7) ? x0 + 1 : x0;
    int y1 = (y0 < 7) ? y0 + 1 : y0;

    float tx = fx - x0;
    float ty = fy - y0;
    float sx = 1.0f - tx;
    float sy = 1.0f - ty;

    ledNextIntensity[y0 * 8 + x0] += sx * sy;
    ledNextIntensity[y0 * 8 + x1] += tx * sy;
    ledNextIntensity[y1 * 8 + x1] += tx * ty;
    ledNextIntensity[y1 * 8 + x0] += sx * ty;
  }

  uint8_t base_r, base_g, base_b;
  button.getCurrentColorRGB(&base_r, &base_g, &base_b);
  int brightness = button.getCurrentBrightness();

  uint8_t scaled_r = (base_r * brightness) / 255;
  uint8_t scaled_g = (base_g * brightness) / 255;
  uint8_t scaled_b = (base_b * brightness) / 255;

  int foam_threshold = button.getCurrentFoamThreshold();

  // 1. Calculate and normalize the current gravity vector orientation
  float g_mag = sqrtf(gravityX * gravityX + gravityY * gravityY);
  float nx = 0.0f;
  float ny = -1.0f; // Default baseline direction
  if (g_mag > 0.001f)
  {
    nx = gravityX / g_mag;
    ny = gravityY / g_mag;
  }

  // 2. Render LEDs using vector dot products mapped dynamically to screen space
  for (int y = 0; y < 8; y++)
  {
    for (int x = 0; x < 8; x++)
    {
      int i = y * 8 + x;
      float previous = ledIntensity[i] * 0.72f;
      float current = ledNextIntensity[i];
      ledIntensity[i] = (current > previous) ? current : previous;

      if (ledIntensity[i] > PARTICLE_THRESHOLD)
      {
        float amount = utils::clamp(ledIntensity[i] / (PARTICLE_THRESHOLD * 2.5f), 0.25f, 1.0f);

        // Map pixel coordinates relative to the 8x8 space center (from -3.5 to +3.5)
        float px = x - 3.5f;
        float py = y - 3.5f;

        // Vector dot product: projects pixel position along the current gravity pull
        float dot_product = px * nx + py * ny;
        float wave_phase = dot_product * 0.22f;

        // Check if the button color is set to White (usually the trigger for Rainbow Mode)
        bool isRainbowMode = (base_r > 240 && base_g > 240 && base_b > 240);

        uint8_t final_r, final_g, final_b;

        if (isRainbowMode)
        {
          // ==========================================
          // MODE 1: Spectral Vector Rainbow
          // ==========================================
          float r_coeff = sinf(wave_phase) * 0.5f + 0.5f;
          float g_coeff = sinf(wave_phase + 2.0944f) * 0.5f + 0.5f;
          float b_coeff = sinf(wave_phase + 4.1888f) * 0.5f + 0.5f;

          final_r = (uint8_t)(brightness * amount * r_coeff);
          final_g = (uint8_t)(brightness * amount * g_coeff);
          final_b = (uint8_t)(brightness * amount * b_coeff);
        }
        else
        {
          // ==========================================
          // MODE 2: Dynamic Solid Colors (From Double Tap)
          // ==========================================
          // Create a natural light/dark shading wave based on gravity pull
          float shading = sinf(wave_phase) * 0.4f + 0.6f;

          // Mix the button's RGB color with the master brightness, particle amount, and shading
          final_r = (uint8_t)((base_r * brightness / 255.0f) * amount * shading);
          final_g = (uint8_t)((base_g * brightness / 255.0f) * amount * shading);
          final_b = (uint8_t)((base_b * brightness / 255.0f) * amount * shading);
        }

        matrix.setPixelColor(i, matrix.Color(final_r, final_g, final_b));
      }
      else if (ledIntensity[i] >= foam_threshold)
      {
        uint8_t foam = (uint8_t)(0.3f * brightness);
        matrix.setPixelColor(i, matrix.Color(foam, foam, foam));
      }
      else
      {
        matrix.setPixelColor(i, matrix.Color(0, 0, 0));
      }
    }
  }

  matrix.show();
}

// read accelerometer and update gravity direction
void updateGravityFromSensor()
{
  float accelX, accelY, accelZ;
  utils::readQMI8658Accel(&accelX, &accelY, &accelZ);

  gravityX = -accelY * GRAVITY_MAGNITUDE;
  gravityY = accelX * GRAVITY_MAGNITUDE;

  // Save the Z-axis reading globally
  currentAccelZ = accelZ;
}

// execute one complete flip timestep with profiling
void runFLIPStep()
{
  unsigned long t0 = micros();

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    particles[i].addGravity(gravityX, gravityY);
    particles[i].updatePosition();
  }
  unsigned long t1 = micros();

  grid.pushParticlesApart(particles, NUM_PARTICLES, 2);
  unsigned long t2 = micros();

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    grid.handleParticleCollision(&particles[i]);
  }
  unsigned long t3 = micros();

  grid.resetCellTypesToAir();
  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    grid.markCellWithLiquid(&particles[i]);
  }

  grid.savePreviousVelocities();
  grid.clearVelocitiesAndWeights();
  unsigned long t4 = micros();

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    grid.transferVelocityfromParticleToGrid(&particles[i]);
  }
  grid.normalizeGridVelocities();
  grid.restoreSolidCellVelocities();
  unsigned long t5 = micros();

  grid.updateParticleDensity(particles, NUM_PARTICLES);
  grid.savePreviousVelocities();
  unsigned long t6 = micros();

  grid.forcingIncompressibility();
  unsigned long t7 = micros();

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    grid.transferVelocityfromGridToParticle(&particles[i]);
  }
  unsigned long t8 = micros();

  t_integrate = (t1 - t0) / 1000.0f;
  t_push = (t2 - t1) / 1000.0f;
  t_collision = (t3 - t2) / 1000.0f;
  t_grid_prep = (t4 - t3) / 1000.0f;
  t_p2g = (t5 - t4) / 1000.0f;
  t_density = (t6 - t5) / 1000.0f;
  t_solve = (t7 - t6) / 1000.0f;
  t_g2p = (t8 - t7) / 1000.0f;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);
  btStop();
  esp_wifi_stop();
  esp_bt_controller_disable();

  setCpuFrequencyMhz(240);

  Serial.println("\n\n================================");
  Serial.println("FLIP Fluid Simulation - ESP32-S3 (8x8)");
  Serial.println("================================\n");

  matrix.begin();
  matrix.clear();
  matrix.show();

  Serial.println("Initializing button...");
  button.init();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  Serial.println("Initializing QMI8658 sensor...");
  if (!utils::initQMI8658())
  {
    Serial.println("ERROR: QMI8658 initialization failed!");
    while (1)
    {
      matrix.clear();
      for (int i = 0; i < 8; i++)
      {
        matrix.setPixelColor(i * 9, matrix.Color(50, 0, 0));
      }
      matrix.show();
      delay(500);
      matrix.clear();
      matrix.show();
      delay(500);
    }
  }
  Serial.println("QMI8658 initialized successfully!");

  Serial.println("Initializing particles...");
  initializeParticles();
  Serial.print("Created ");
  Serial.print(NUM_PARTICLES);
  Serial.println(" particles");

  Serial.println("Initializing spatial hash grid...");
  grid.initParticleSpatialHash(NUM_PARTICLES, particleRadius);
  grid.markCellWalls();

  Serial.println("\n=== Configuration ===");
  Serial.printf("Grid size: %d x %d\n", GRID_SIZE_X, GRID_SIZE_Y);
  Serial.printf("Physical size: %.2f x %.2f mm\n", PHYSICAL_WIDTH * 1000.0f, PHYSICAL_HEIGHT * 1000.0f);
  Serial.printf("Timestep: %.1f ms\n", DELTA_T * 1000.0f);
  Serial.printf("Target FPS: %.1f\n", 1.0f / DELTA_T);
  Serial.printf("FLIP ratio: %.2f\n", FLIP_RATIO);
  Serial.printf("Incompressibility iterations: %d\n", INCOMPRESSIBILITY_ITERATIONS);

  Serial.println("\n=== Starting simulation ===");
  lastStepTime = millis();
}

void loop()
{
  unsigned long currentTime = millis();
  const unsigned long stepInterval = (unsigned long)(FRAME_INTERVAL * 1000.0f);
  static unsigned long accumulatedTime = 0;
  const unsigned long maxFrameCatchup = stepInterval * 2;

  button.update();

  // --- Face-Down Sleep Timer Logic ---
  static unsigned long faceDownStartTime = 0;
  static bool isFaceDown = false;

  // Check if Z-axis indicates face-down.
  // Depending on how your IMU chip is mounted, face-down will either be < -0.8f or > 0.8f.
  // If it goes to sleep while face-UP, simply change this to: (currentAccelZ > 0.8f)
  Serial.printf("Current Z-axis: %.3f\n", currentAccelZ);
  if (currentAccelZ > 0.2f)
  {
    if (!isFaceDown)
    {
      isFaceDown = true;
      faceDownStartTime = millis(); // Start the 3-second stopwatch
      Serial.println("Face down detected - starting sleep timer...");
    }
    else if (millis() - faceDownStartTime >= 3000)
    {
      // 3 seconds have passed! Go to sleep.
      enterSleepMode();
      isFaceDown = false; // Reset state when we wake back up
    }
  }
  else
  {
    // We picked it up before 3 seconds finished, cancel the timer.
    if (isFaceDown)
    {
      isFaceDown = false;
      Serial.println("Sleep timer canceled.");
    }
  }

  accumulatedTime += currentTime - lastStepTime;
  lastStepTime = currentTime;
  if (accumulatedTime > maxFrameCatchup)
  {
    accumulatedTime = maxFrameCatchup;
  }

  if (accumulatedTime >= stepInterval)
  {
    accumulatedTime -= stepInterval;

    updateGravityFromSensor();
    runFLIPStep();

    unsigned long t_vis_start = micros();
    visualizeParticles();
    t_vis = (micros() - t_vis_start) / 1000.0f;

    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 10000)
    {
      float total_ms = t_integrate + t_push + t_collision + t_grid_prep + t_p2g + t_density + t_solve + t_g2p + t_vis;
      float max_fps = 1000.0f / total_ms;

      Serial.println("\n--- Performance Stats ---");
      Serial.printf("Total Frame Time: %.2f ms | Max FPS: %.1f\n", total_ms, max_fps);
      Serial.println("Breakdown:");
      Serial.printf("  1. Integrate: %.2f ms\n", t_integrate);
      Serial.printf("  2. Separation:%.2f ms\n", t_push);
      Serial.printf("  3. Collision: %.2f ms\n", t_collision);
      Serial.printf("  4. Grid Prep: %.2f ms\n", t_grid_prep);
      Serial.printf("  5. P -> G:    %.2f ms\n", t_p2g);
      Serial.printf("  6. Density:   %.2f ms\n", t_density);
      Serial.printf("  7. Solver:    %.2f ms\n", t_solve);
      Serial.printf("  8. G -> P:    %.2f ms\n", t_g2p);
      Serial.printf("  9. Visualize: %.2f ms\n", t_vis);

      lastPrintTime = currentTime;
    }
  }
}