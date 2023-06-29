import os
import sys
import tensorrt as trt

Logger = trt.Logger(trt.Logger.WARNING)

def build_engine_onnx(path):
    builder = trt.Builder(Logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.set_flag(trt.BuilderFlag.FP16) # reduces inference time, but may decrease accuracy
    parser = trt.OnnxParser(network, Logger)

    config.max_workspace_size = 1 << 20
    # Load the Onnx model and parse it in order to populate the TensorRT network.
    with open(path, 'rb') as model_file:
        success = parser.parse_from_file(path)
        if not success:
            print ('ERROR: failed to parse the ONNX file.')
            for error in range(parser.num_errors):
                print (parser.get_error(error))
            return None
    return builder.build_serialized_network(network, config)


def main(model_path):
    serialized_engine = build_engine_onnx(model_path)

    engine_path = os.path.splitext(model_path)[0] + '.trt'
    print('saving serialized engine to ' + engine_path)

    with open(engine_path, 'wb') as f:
        f.write(serialized_engine)

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('ERROR: please specify model path')
        exit()

    model_path = sys.argv[1]
    print('reading model from ' + model_path)

    if not os.path.isfile(model_path):
        print('ERROR: specified model file was not found')
        exit()
    
    main(model_path)