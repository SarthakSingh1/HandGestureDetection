#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// Updated Distortion Parameters for controlled soft clipping
float drive = 10.0f; // Controls the amount of distortion
float wet = 0.5f; // Mix between dry (original) and wet (distorted) signal

float applyDistortion(float sample) {
    // Apply drive
    sample *= drive;

    // Soft clipping using a tanh function for smoother transitions
    sample = tanh(sample);

    // Mix the original (dry) and distorted (wet) signals
    return sample * wet + sample / drive * (1.0 - wet);
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
        float processedSample = applyDistortion(inputSample);
        int16_t outputSample = (int16_t)(processedSample * 32767.0f);
        fwrite(&outputSample, sizeof(int16_t), 1, out);
    }

    fclose(in);
    fclose(out);
}

int main() {
    processWavFile("doom_mono.wav", "doom_mono_soft_distorted.wav");
    return 0;
}
