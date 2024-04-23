#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define SAMPLE_RATE 44100 // Typical sample rate for audio files
#define REVERB_BUFFER_SIZE 4410 // 100ms delay at 44.1kHz
#define REVERB_FEEDBACK 0.5f
#define REVERB_MIX 0.3f

float reverbBuffer[REVERB_BUFFER_SIZE] = {0};
int reverbWriteIndex = 0;

float applyReverb(float inputSample) {
    // Calculate the read index
    int readIndex = reverbWriteIndex - REVERB_BUFFER_SIZE / 2;
    if (readIndex < 0) readIndex += REVERB_BUFFER_SIZE;

    // Read the delayed sample and apply feedback
    float delayedSample = reverbBuffer[readIndex] * REVERB_FEEDBACK;

    // Write the new sample to the buffer
    reverbBuffer[reverbWriteIndex++] = inputSample + delayedSample;

    // Wrap the write index
    if (reverbWriteIndex >= REVERB_BUFFER_SIZE) reverbWriteIndex = 0;

    // Mix the original and delayed samples
    return inputSample * (1.0f - REVERB_MIX) + delayedSample * REVERB_MIX;
}

void processWavFile(const char* inputFile, const char* outputFile) {
    FILE *in = fopen(inputFile, "rb");
    FILE *out = fopen(outputFile, "wb");
    if (!in || !out) {
        perror("Error opening file");
        return;
    }

    // Copy header to output file
    uint8_t header[44];
    fread(header, sizeof(uint8_t), 44, in);
    fwrite(header, sizeof(uint8_t), 44, out);

    int16_t buffer;
    while (fread(&buffer, sizeof(int16_t), 1, in) == 1) {
        float inputSample = buffer / 32768.0f;
        float processedSample = applyReverb(inputSample);
        int16_t outputSample = (int16_t)(processedSample * 32767.0f);
        fwrite(&outputSample, sizeof(int16_t), 1, out);
    }

    fclose(in);
    fclose(out);
}

int main() {
    processWavFile("doom_mono.wav", "doom_mono_reverb.wav");
    return 0;
}
