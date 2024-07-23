// music.h
#ifndef MUSIC_H
#define MUSIC_H

#include <Arduino.h>

// Define musical notes
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// Define the success melody (notes and durations)
const int successMelody[] = {
  NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_G4, NOTE_E4, NOTE_C4
};

const int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 4
};

void playMusic(int buzzerChannel) {
  int noteCount = sizeof(successMelody) / sizeof(successMelody[0]);
  for (int i = 0; i < noteCount; i++) {
    int noteDuration = 1000 / noteDurations[i];
    ledcWriteTone(buzzerChannel, successMelody[i]); // Play the note
    delay(noteDuration);
    ledcWriteTone(buzzerChannel, 0); // Stop the tone
    delay(noteDuration * 0.3); // Pause between notes
  }
  ledcWriteTone(buzzerChannel, 0); // Ensure the tone is off after playing the melody
}

#endif // MUSIC_H
