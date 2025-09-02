#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dsp.h"
#include "time_machine_hardware.h"

using namespace daisy;
using namespace oam;
using namespace time_machine;
using namespace std;

#define TIME_SECONDS 150
#define BUFFER_WIGGLE_ROOM_SAMPLES 1000

#define LINEAR_TIME false

//Setting Struct containing parameters we want to save to flash
struct CalibrationData {
	float timeCvOffset = 0.0;
	float skewCvOffset = 0.0;
	float feedbackCvOffset = 0.0;
	int calibrated = false;

	//Overloading the != operator
	//This is necessary as this operator is used in the PersistentStorage source code
	bool operator!=(const CalibrationData& a) const {
        return !(
				a.timeCvOffset==skewCvOffset && \
				a.skewCvOffset==skewCvOffset && \
				a.feedbackCvOffset==feedbackCvOffset && \
				a.calibrated==calibrated
			);
    }
};

// init buffers - add an extra second just in case we somehow end up slightly beyond max time
// due to precision loss in floating point arithmetic (maybe use doubles for time values???)
float DSY_SDRAM_BSS bufferLeft[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];
float DSY_SDRAM_BSS bufferRight[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];

TimeMachineHardware hw;
PersistentStorage<CalibrationData> CalibrationDataStorage(hw.qspi);
GateIn gate;
Led leds[9];

StereoTimeMachine timeMachine;
ClockRateDetector clockRateDetector;
ContSchmidt timeKnobSchmidt;
ContSchmidt timeCvSchmidt;

Slew timeKnobSlew;
Slew feedbackKnobSlew;
Slew distributionKnobSlew;
Slew timeCvSlew;
Slew feedbackCvSlew;
Slew distributionCvSlew;

// global storage for CV/knobs so we don't get them twice to print diagnostics
float timeCv = 0.0;
float feedbackCv = 0.0;
float skewCv = 0.0;
float timeKnob = 0.0;
float feedbackKnob = 0.0;
float skewKnob = 0.0;
float drySlider = 0.0;
float delaySliders = 0.0;

float vcaIn[9];

// calibration offsets for CV
float timeCvOffset = 0.0;
float feedbackCvOffset = 0.0;
float skewCvOffset = 0.0;

float finalTimeValue = 0.0;
float finalDistributionValue = 0.0;
float finalFeedbackValue = 0.0;

// delay setting LEDs for startup sequences
bool setLeds = false;

CpuLoadMeter cpuMeter;

int droppedFrames = 0;

// called every N samples (search for SetAudioBlockSize)
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// cpu meter measurements start
	cpuMeter.OnBlockStart();
	droppedFrames++;

	// process controls
	hw.ProcessAllControls();

	// populate/update global CV/knob vars (time is slewed to reduce noise at large time values)
	timeKnob = minMaxKnob(1.0 - hw.GetAdcValue(TIME_KNOB), 0.0008);
	feedbackKnob = fourPointWarp(1.0 - minMaxKnob(hw.GetAdcValue(FEEDBACK_KNOB), 0.028));
	skewKnob = fourPointWarp(1.0 - minMaxKnob(hw.GetAdcValue(SKEW_KNOB), 0.0008));

	timeCv = clamp(hw.GetAdcValue(TIME_CV) - timeCvOffset, -1, 1);
	feedbackCv = clamp(hw.GetAdcValue(FEEDBACK_CV) - feedbackCvOffset, -1, 1);
	skewCv = clamp(hw.GetAdcValue(SKEW_CV) - skewCvOffset, -1, 1);
	vcaIn[0] = clamp(hw.GetAdcValue(DRY_VCA), 0, 1);
	vcaIn[1] = 1.0;
	vcaIn[2] = 1.0;
	vcaIn[3] = 1.0;
	vcaIn[4] = 1.0;
	vcaIn[5] = 1.0;
	vcaIn[6] = 1.0;
	vcaIn[7] = 1.0;
	vcaIn[8] = clamp(hw.GetAdcValue(CV_7), 0, 1);
	
	drySlider = minMaxSlider(1.0 - hw.GetAdcValue(DRY_SLIDER));

	// calculate time based on clock if present, otherwise simple time
	float time = 0.0; 
	if(clockRateDetector.GetInterval() > 0.0) {
		// 12 quantized steps for knob, 10 for CV (idk what these quanta should actually be)
		// time doubles and halves with each step, they are additive/subtractive
		float timeCoef = pow(2.0, (timeKnobSchmidt.Process((1.0-timeKnob)*12)) + (timeCvSchmidt.Process(timeCv*10))) / pow(2.0, 6.0);
		time = clockRateDetector.GetInterval() / timeCoef;
		// make sure time is a power of two less than the max time available in the buffer
		while(time > TIME_SECONDS) time *= 0.5;
	} else {
		// time linear with knob, scaled v/oct style with CV
		time = pow(timeKnobSlew.Process(timeKnob), 2.0) * 8.0 / pow(2.0, timeCvSlew.Process(timeCv) * 5.0);
	}

	// force time down to a max value (taking whichever is lesser, the max or the time)
	time = std::min((float)TIME_SECONDS, time);
	// condition feedback knob to have deadzone in the middle, add CV
	float feedback = clamp(fourPointWarp(feedbackKnobSlew.Process(feedbackKnob)) * 2.0 + feedbackCvSlew.Process(feedbackCv), 0, 3);
	// condition distribution knob value to have deadzone in the middle, add CV
	float distribution = fourPointWarp(distributionKnobSlew.Process(skewKnob)) + distributionCvSlew.Process(skewCv);

	finalTimeValue = time;
	finalFeedbackValue = feedback;
	finalDistributionValue = distribution;

	for(int i=0; i<9; i++) {
		if(i<8) {
			// set LEDs based on loudness for last 8 sliders
			float loudness = timeMachine.timeMachineLeft.readHeads[i].loudness.Get();
			loudness = max(loudness, timeMachine.timeMachineRight.readHeads[i].loudness.Get());
			if(setLeds) {
				leds[i+1].Set(loudness);
				leds[i+1].Update();
			}
		} else {
			// set LEDs based on loudness for first slider
			float loudness = timeMachine.timeMachineLeft.loudness.Get();
			loudness = max(loudness, timeMachine.timeMachineRight.loudness.Get());
			if(setLeds) {
				leds[0].Set(loudness);
				leds[0].Update();
			}
		}
	}

	// set time machine dry slider value, feedback, "blur" which is semi-deprecated
	timeMachine.Set(drySlider * hw.GetVcaValue(0), feedback, feedback); // controlling "blur" with feedback now???

	for(int i=1; i<9; i++) {
		// let last 8 slider time/amp/blur values for left channel time machine instance
        timeMachine.timeMachineLeft.readHeads[i-1].Set(
            spread((i / 8.0), distribution) * time,
            max(0.0f, minMaxSlider((1.0f - hw.GetSliderValue(i)) * hw.GetVcaValue(i))),
						max(0., feedback-1.0)
        );
		// let last 8 slider time/amp/blur values for right channel time machine instance
		timeMachine.timeMachineRight.readHeads[i-1].Set(
            spread((i / 8.0), distribution) * time,
            max(0.0f, minMaxSlider((1.0f - hw.GetSliderValue(i)) * hw.GetVcaValue(i))),
						max(0., feedback-1.0)
        );
	}

	for (size_t i = 0; i < size; i++)
	{
		// process gate for clock rate detector at audio rate (per-sample) so it calculates clock correctly
		clockRateDetector.Process(hw.gate_in_2.State());
		// process input into time machine
		float* output = timeMachine.Process(in[0][i], in[1][i]);
		// set hardware output to time machine output
		out[0][i] = output[0];
		out[1][i] = output[1];
	}

	//cpu meter measurement stop
	cpuMeter.OnBlockEnd();
	droppedFrames--;
}

bool shouldCalibrate() {
		bool shouldCalibrate = \
			(hw.GetAdcValue(SKEW_CV) < 0.01) && \
			(hw.GetAdcValue(TIME_CV) < 0.01) && \
			(hw.GetAdcValue(FEEDBACK_CV) < 0.01) && \
			hw.gate_in_2.State();
		for(int i=0; i<9; i++) {
			shouldCalibrate &= hw.GetSliderValue(i) < 0.01;
		}
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(TIME_KNOB)) > 0.95;
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(SKEW_KNOB)) > 0.95;
		shouldCalibrate &= minMaxKnob(1.0 - hw.GetAdcValue(FEEDBACK_KNOB)) > 0.95;
		return shouldCalibrate;
}

int main(void)
{
	// init time machine hardware
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback

	dsy_gpio_pin gatePin = DaisyPatchSM::B9;
	gate.Init(&gatePin);

	// initialize LEDs
	leds[0].Init(DaisyPatchSM::D1, false);
	leds[1].Init(DaisyPatchSM::D2, false);
	leds[2].Init(DaisyPatchSM::D3, false);
	leds[3].Init(DaisyPatchSM::D4, false);
	leds[4].Init(DaisyPatchSM::D5, false);
	leds[5].Init(DaisyPatchSM::A9, false);
	leds[6].Init(DaisyPatchSM::D10, false);
	leds[7].Init(DaisyPatchSM::D7, false);
	leds[8].Init(DaisyPatchSM::D6, false);

	// set sample rate
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// init slew limiter for time (we should tune this more delibrately)
	timeKnobSlew.Init(0.5, 0.0005);
	feedbackKnobSlew.Init(0.5, 0.0005);
	distributionKnobSlew.Init(0.5, 0.0005);
	timeCvSlew.Init(0.5, 0.0005);
	feedbackCvSlew.Init(0.5, 0.0005);
	distributionCvSlew.Init(0.5, 0.0005);

	// init clock rate detector
	clockRateDetector.Init(hw.AudioSampleRate());

	// init time machine
    timeMachine.Init(hw.AudioSampleRate(), TIME_SECONDS + (((float)BUFFER_WIGGLE_ROOM_SAMPLES) * 0.5 / hw.AudioSampleRate()), bufferLeft, bufferRight);

	// load calibration data, using sensible defaults
	CalibrationDataStorage.Init({0.0f, 0.0f, 0.0f, false});
	CalibrationDataStorage.GetSettings();
	CalibrationData &savedCalibrationData = CalibrationDataStorage.GetSettings();

	// init cpu meter
	cpuMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

	// start time machine hardware audio and logging
    hw.StartAudio(AudioCallback);

	// LED startup sequence
	int ledSeqDelay = 100;
	for(int i=0; i<9; i++) {
		for(int j=0; j<9; j++) {
			leds[j].Set(j == i ? 1.0 : 0.0);
			leds[j].Update();
		}
		hw.PrintLine("%d", i);
		System::Delay(ledSeqDelay);
	}

	if(shouldCalibrate()) {

		bool calibrationReady = true;

		// do reverse LED startup sequence while
		// checking that we definitely want to calibrate
		for(int i=0; i<(5000/ledSeqDelay); i++) {
			for(int j=0; j<9; j++) {
				leds[j].Set(j == (8 - (i%9)) ? 1.0 : 0.0);
				leds[j].Update();
			}
			System::Delay(ledSeqDelay);
			calibrationReady &= shouldCalibrate();
			if(!calibrationReady) break;
		}
		
		if(calibrationReady) {
			// perform calibration routine
			int numSamples = 128;
			for(int i = 0; i < numSamples; i++) {
				// accumulate cv values
				savedCalibrationData.timeCvOffset += timeCv;
				savedCalibrationData.skewCvOffset += skewCv;
				savedCalibrationData.feedbackCvOffset += feedbackCv;
				// wait 10ms
				System::Delay(10);
				// set LEDs
				for(int ledIndex=0; ledIndex<9; ledIndex++) {
					leds[ledIndex].Set(i % 8 < 4 ? 1.0f : 0.0f);
					leds[ledIndex].Update();
				}
			}
			
			// divide CVs by number of samples taken to get average
			savedCalibrationData.timeCvOffset = savedCalibrationData.timeCvOffset / ((float)numSamples);
			savedCalibrationData.skewCvOffset = savedCalibrationData.skewCvOffset / ((float)numSamples);
			savedCalibrationData.feedbackCvOffset = savedCalibrationData.feedbackCvOffset / ((float)numSamples);
			
			// set calibrated value to true
			savedCalibrationData.calibrated = true;
			
			// save calibration data
			CalibrationDataStorage.Save();
		}
	}

	timeCvOffset = savedCalibrationData.timeCvOffset;
	skewCvOffset = savedCalibrationData.skewCvOffset;
	feedbackCvOffset = savedCalibrationData.feedbackCvOffset;

	hw.StartLog();

	setLeds = true;

	while(1) {

		// print diagnostics
		hw.PrintLine("TIME_CV: " FLT_FMT(6), FLT_VAR(6, timeCv));
		hw.PrintLine("FEEDBACK_CV: " FLT_FMT(6), FLT_VAR(6, feedbackCv));
		hw.PrintLine("SKEW_CV: " FLT_FMT(6), FLT_VAR(6, skewCv));
		hw.PrintLine("TIME_KNOB: " FLT_FMT(6), FLT_VAR(6, timeKnob));
		hw.PrintLine("FEEDBACK_KNOB: " FLT_FMT(6), FLT_VAR(6, feedbackKnob));
		hw.PrintLine("SKEW_KNOB: " FLT_FMT(6), FLT_VAR(6, skewKnob));
		hw.PrintLine("GATE IN: %d", hw.gate_in_2.State());

		hw.PrintLine("GATE IN: %d", hw.gate_in_1.State());
		hw.PrintLine("CV IN 1: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_4)));
		hw.PrintLine("CV IN 2: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_5)));
		hw.PrintLine("CV IN 3: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_6)));
		hw.PrintLine("CV IN 4: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(CV_7)));

		hw.PrintLine("TIME_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.timeCvOffset));
		hw.PrintLine("FEEDBACK_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.feedbackCvOffset));
		hw.PrintLine("SKEW_CAL: " FLT_FMT(6), FLT_VAR(6, savedCalibrationData.skewCvOffset));
		hw.PrintLine("CALIBRATED: %d", savedCalibrationData.calibrated);

		hw.PrintLine("FINAL TIME: " FLT_FMT(6), FLT_VAR(6, finalTimeValue));
		hw.PrintLine("FINAL DISTRIBUTION: " FLT_FMT(6), FLT_VAR(6, finalDistributionValue));
		hw.PrintLine("FINAL FEEDBACK: " FLT_FMT(6), FLT_VAR(6, finalFeedbackValue));

		hw.PrintLine("CPU AVG: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetAvgCpuLoad()));
		hw.PrintLine("CPU MIN: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMinCpuLoad()));
		hw.PrintLine("CPU MAX: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMaxCpuLoad()));

		hw.PrintLine("DROPPED FRAMES: %d", droppedFrames);

		for(int i=0; i<9; i++) {
			hw.PrintLine("%d: " FLT_FMT(6), i, FLT_VAR(6, minMaxSlider(1.0 - hw.GetSliderValue(i))));
		}

		hw.PrintLine("");
		System::Delay(250);
	}
}
