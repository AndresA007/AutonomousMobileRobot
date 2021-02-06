#!/bin/bash

python3 $AUTONOMOUS_MOBILE_ROBOT_HOME/DeepSpeechModule/src/vva_deepspeech_v0.8.2_server_main.py \
  --audio $AUTONOMOUS_MOBILE_ROBOT_HOME/VVA_ws/src/vva_voice_interact_server/records/rcvd_temp.wav \
  --model $AUTONOMOUS_MOBILE_ROBOT_HOME/DeepSpeechModule/english_model_v0.8.2 \
  --port 39567

