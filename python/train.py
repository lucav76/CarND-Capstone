import net
import aug

light_net = net.LightNet(["C:\\datasets\\Udacity\\object-dataset\\Additional\\", "C:\\datasets\\Udacity\\object-dataset\\Simulator\\", "C:\\datasets\\Udacity\\object-dataset\\manualRgb\\"], False)
light_net.create_dataset()

light_net.show_distribution()
light_net.create_model()
light_net.train(True)

