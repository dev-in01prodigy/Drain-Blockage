struct BatchResult {
  uint8_t stage;
  uint8_t param;
  uint8_t valIdx;
  double  mean;
  double  stddev;
  uint8_t b6;
};

BatchResult results[26];
int resultCount = 0;

BatchResult* findResult(uint8_t stage, uint8_t param, uint8_t valIdx) {
  for (int i = 0; i < resultCount; i++)
    if (results[i].stage == stage) return &results[i];
  return NULL;
}

void setup() {}
void loop() {}
