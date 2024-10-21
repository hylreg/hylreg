#include <yolo_v8.h>

void MyYOLO_V8::Detector(YOLO_V8*& p,cv::Mat &img) {
    // std::filesystem::path current_path = std::filesystem::current_path();
    // std::filesystem::path imgs_path = current_path / "images";

    // for (auto& i : std::filesystem::directory_iterator(imgs_path))
    // {
    //     if (i.path().extension() == ".jpg" || i.path().extension() == ".png" || i.path().extension() == ".jpeg")
    //     {
    //         std::string img_path = i.path().string();
    //         img = cv::imread(img_path);
            std::vector<DL_RESULT> res;
            int64 start = cv::getTickCount();
            p->RunSession(img, res);

            for (auto& re : res)
            {
                cv::RNG rng(cv::getTickCount());
                cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

                cv::rectangle(img, re.box, color, 3);

                float confidence = floor(100 * re.confidence) / 100;
                std::cout << std::fixed << std::setprecision(2);
                std::string label = p->classes[re.classId] + " " +
                                    std::to_string(confidence).substr(0, std::to_string(confidence).size() - 4);

                cv::rectangle(
                    img,
                    cv::Point(re.box.x, re.box.y - 25),
                    cv::Point(re.box.x + label.length() * 15, re.box.y),
                    color,
                    cv::FILLED
                    );

                cv::putText(
                    img,
                    label,
                    cv::Point(re.box.x, re.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.75,
                    cv::Scalar(0, 0, 0),
                    2
                    );


            }

            // 计算FPS render it
            float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
            putText(img, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 0), 2, 8);

            // std::cout << "Press any key to exit" << std::endl;
            // cv::imshow("Result of Detection", img);
            // cv::waitKey(30);
            // cv::destroyAllWindows();
        // }
    // }
}

void MyYOLO_V8::Classifier(YOLO_V8*& p)
{
    std::filesystem::path current_path = std::filesystem::current_path();
    std::filesystem::path imgs_path = current_path;// / "images"
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);
    for (auto& i : std::filesystem::directory_iterator(imgs_path))
    {
        if (i.path().extension() == ".jpg" || i.path().extension() == ".png")
        {
            std::string img_path = i.path().string();
            //std::cout << img_path << std::endl;
            cv::Mat img = cv::imread(img_path);
            std::vector<DL_RESULT> res;
            char* ret = p->RunSession(img, res);

            float positionY = 50;
            for (int i = 0; i < res.size(); i++)
            {
                int r = dis(gen);
                int g = dis(gen);
                int b = dis(gen);
                cv::putText(img, std::to_string(i) + ":", cv::Point(10, positionY), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(b, g, r), 2);
                cv::putText(img, std::to_string(res.at(i).confidence), cv::Point(70, positionY), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(b, g, r), 2);
                positionY += 50;
            }

            cv::imshow("TEST_CLS", img);
            cv::waitKey(0);
            cv::destroyAllWindows();
            //cv::imwrite("E:\\output\\" + std::to_string(k) + ".png", img);
        }

    }
}

int MyYOLO_V8::ReadCocoYaml(YOLO_V8*& p,QString classPath) {
    // Open the YAML file
    // std::ifstream file("../../coco8.yaml");
    std::ifstream file("/"+classPath.toStdString());


    if (!file.is_open())
    {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }

    // Read the file line by line
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line))
    {
        lines.push_back(line);
    }

    // Find the start and end of the names section
    std::size_t start = 0;
    std::size_t end = 0;
    for (std::size_t i = 0; i < lines.size(); i++)
    {
        if (lines[i].find("names:") != std::string::npos)
        {
            start = i + 1;
        }
        else if (start > 0 && lines[i].find(':') == std::string::npos)
        {
            end = i;
            break;
        }
    }

    // Extract the names
    std::vector<std::string> names;
    for (std::size_t i = start; i < end; i++)
    {
        std::stringstream ss(lines[i]);
        std::string name;
        std::getline(ss, name, ':'); // Extract the number before the delimiter
        std::getline(ss, name); // Extract the string after the delimiter
        names.push_back(name);
    }

    p->classes = names;
    return 0;
}

void MyYOLO_V8::CreateDetectSession(YOLO_V8*& p,QString modelPath,QString classPath)
{
//     YOLO_V8 *yoloDetector = new YOLO_V8;
    ReadCocoYaml(p,classPath);
    DL_INIT_PARAM params;
    params.rectConfidenceThreshold = 0.6;
    params.iouThreshold = 0.5;
//     params.modelPath = "../YOLOv8-ONNXRuntime-CPP/yolov8n.onnx";
    params.modelPath = "/"+modelPath.toStdString();

    params.imgSize = { 640, 640 };
#ifdef USE_CUDA
    params.cudaEnable = true;

    // GPU FP32 inference
    params.modelType = YOLO_DETECT_V8;
    // GPU FP16 inference
    //Note: change fp16 onnx model
    //params.modelType = YOLO_DETECT_V8_HALF;

#else
    // CPU inference
    params.modelType = YOLO_DETECT_V8;
    params.cudaEnable = false;

#endif
    p->CreateSession(params);

    std::cout<<"CreateSession"<<std::endl;
//    Detector(yoloDetector);
}

// void MyYOLO_V8::ClsTest()
// {
//     YOLO_V8* yoloDetector = new YOLO_V8;
//     std::string model_path = "cls.onnx";
//     ReadCocoYaml(yoloDetector);
//     DL_INIT_PARAM params{ model_path, YOLO_CLS, {224, 224} };
//     yoloDetector->CreateSession(params);
//     Classifier(yoloDetector);
// }



// int main()
// {
//     DetectTest();
//     // ClsTest();
// }
