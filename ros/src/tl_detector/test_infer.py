import infer
import utils

print("GPU found: " + utils.get_gpu_name())
ldac = infer.LightDetectionAndClassification(detection_model = infer.SSD_MOBILE_NET)

desired_labels=["Red", "Green", "Yellow"]

files = ["traffic.jpg", "traffic2.jpg", "traffic3.jpg", "traffic4.jpg", "traffic5.jpg",
         "traffic6.jpg", "traffic7.jpg", "traffic8.jpg", "traffic9.jpg", "traffic10.jpg",
         "left0000.jpg", "left0003.jpg", "left0011.jpg", "left0027.jpg", "left0140.jpg", "left0701.jpg"]

for file in files:
    print("\n\n\n", file)
    ldac.infer_and_save(file, desired_labels=desired_labels, confidence_cutoff=0.6)

