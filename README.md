# Music Synthesizer
The <b>Music Synthesizer</b> project is a project which focuses on real-time system analysis and concurrency to create an embedded software for a
musical synthesizer. The keyboard module is controlled using a ST NUCLEO-L432KC microcontroller that contains a STM32L432KCU6U processor, 
which has an Arm Cortex-M4 core. The functionalities achieved in the module are described below:

## Core Specifications and Functionality
<ol>
  <li>The synthesizer plays the appropriate musical tone using a sawtooth wave upon key press</li>
  <li>There is no prceptible delay between pressing a key and playing the tone</li>
  <li>There is a volume control with 16 increments</li>
  <li>OLED display shows the note in play and volume level</li>
  <li>Synthesizer sends a message to the serial port upon key press or release</li>
  <li>Synthesizer stops / plays a note when receiving an appropriate message on the serial port</li>
  <li>Serial port message for playing a note is of the form Pxy, where x represents the octave number and y is the note number in hexadecimal format</li>
  <li>Serial port message for releasing a note is of the form Rxy, where x represents the octave number and y is the note number in hexadecimal format</li>
</ol>

## Non-functional Specifications
<ol>
  <li>
    The system is implemented using interrupts and threads to achieve concurrent execution of tasks. This can be seen in the table below
    <table align="center">
      <tr>
        <th align=center>Task Name</th>
        <th align=center>Task Type</th>
        <th align=center>Priority</th>
      </tr>
      <tr>
        <td align=center>sampleISR</td>
        <td align=center>Interrupt</td>
        <td align=center>-</td>
      </tr>
      <tr>
        <td align=center>scanKeysTask</td>
        <td align=center>Threading</td>
        <td align=center>3</td>
      </tr>
      <tr>
        <td align=center>displayUpdateTask</td>
        <td align=center>Threading</td>
        <td align=center>4</td>
      </tr>
      <tr>
        <td align=center>sampleWriteTask</td>
        <td align=center>Threading</td>
        <td align=center>2</td>
      </tr>
      <tr>
        <td align=center>msgInTask</td>
        <td align=center>Threading</td>
        <td align=center>1</td>
      </tr>
      <tr>
        <td align=center>msgOutTask</td>
        <td align=center>Threading</td>
        <td align=center>3</td>
      </tr>
    </table>
  </li>
  <li>All volatile data and resources are protected by mutex (for mutually-exclusive access) and atomic load operations. The list of mutexes used to access different variables are shown below</li>

```Python
keyArrayMutex = xSemaphoreCreateMutex();
wetMutex = xSemaphoreCreateMutex();
doubleBufferOneSemaphore = xSemaphoreCreateBinary();
doubleBufferTwoSemaphore = xSemaphoreCreateBinary();
knobThreadSafeMutex = xSemaphoreCreateMutex();
polyArrayMutex = xSemaphoreCreateMutex();
```

  <li>The code can be viewed <a href=/instrumentedSynth/instrumentedSynth.ino>here.</a></li>
</ol>

## Advanced Features
The following advanced features are added to the music synthesizer:
<ol>
  <li><p><b>Sine Wave Musical Tone: </b>
  In addition to producing musical tones on a sawtooth wave function, the synthesizer allows users to toggle to tones produced with sine waves by pressing knob 3 (right-most
  knob). This 'sine wave' mode is displayed in the OLED display. This functionality uses a global array (table) to store the sine wave sample values. Different frequency tones     will sample this array at different step sizes. The higher the frequency, the larger the step size between each sample (sampled at the sampleISR rate of 22kHz).
  </p></li>
  
  <li><p><b>Polyphony: </b>
  In addition to the core functionality of playing musical notes based on a sawtooth function, this synthesizer implementation allows multiple notes (up to 3) to be 
  played simultaneously through the single audio channel. This is performed through dynamic time-multiplexing, where each note is allocated a dynamic time-interval to play the
  note. Three distinct step-sizes are stored as a volatile global variable to hold information of up to 3 keys being pressed simultaneously. The phase accumulator will be 
  incremented by these distinct key values at their allocated time intervals, giving a polyphonic note. In addition, an array 'polyFrequencies' is used to store all keys 
  currently in play.

```C++
volatile uint32_t polyFrequencies [] = {NULL, NULL, NULL};
volatile uint32_t currentStepSize_0 = 0;
volatile uint32_t currentStepSize_1 = 0;
volatile uint32_t currentStepSize_2 = 0;
```

  The effect of this polyphony implementation can be observed in the images below:
  <p align="center">
    <img src="/img/polyphony1.PNG" width="550" alt="Polyphony Notes">
  </p>
  <p align="center">
    <img src="/img/polyphony2.PNG" width="750" alt="Polyphony Effect">
  </p>
  
  Lastly, this advanced feature also works alongside the core functionality of writing to the serial port / reading from the serial port when a key is pressed or released.         However, as the polyphony feature allows at max 3 keys being played simultaneously, any additional keys pressed after 3 keys are already in play will be ignored.
  </p></li>
  
  <li><p><b>Reverberation: </b>
  reverberation is a complex effect resulting from multiple reflections of soundwaves from nearby surfaces resulting in the sound waves superposing and arriving at the ear 
  at different times (see below). This effect can be toggled by pressing knob 0 (left-most knob), and the amount of reverberation can be adjusted by rotating knob 0.
  
  <p align="center">
    <img src="/img/reverb.png" width="450" alt="Reverberation">
  </p>
  
  This effect cannot be accurately recreated with a simple delay so instead a Schroeder Reverberator was implemented to simulate the effect, which requires the use of an allpass filter and a comb filter. The filter network's goal is to create a series of dense echoes that decay over time. Each filter is shown in the diagrams below:
  
  <p align="center">
    <img src="/img/schro_filter.PNG" width="700" alt="Schroeder Reverberator">
  </p>
  <p align="center">
    <img src="/img/comb_filter.PNG" width="480" alt="Comb Filter">
    <img src="/img/allpass_filter.PNG" width="431" alt="Allpass Filter">
  </p>
  </li>
  
</ol>

  


