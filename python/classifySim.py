import infer

ldac = infer.LightDetectionAndClassification()

desired_labels=["Red", "Green", "Yellow", "RedLeft", "GreenLeft", "YellowLeft"]
ldac.infer_and_save_dir("C:\\datasets\\Udacity\\object-dataset-sim\\sim_training_data\\sim_data_capture", desired_labels=desired_labels)
#ldac.infer_and_save_dir("C:\\datasets\\Udacity\\object-dataset-2")

