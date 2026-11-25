#include "HighScoreTypes.hpp"
#include "ImageSource.hpp"
#include "TagDetector.hpp"
#include "SerialLink.hpp"
#include "Visualizer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

using namespace std;

// ---------------------------------------------------------------------------
// 构建标记：用来一眼确认跑的是不是最新代码
// ---------------------------------------------------------------------------
constexpr const char* kBuildId   = "RMHS-2026-11-26-CONT-2";
constexpr const char* kBuildNote = "连续识别 + 串口发送 + 日志";
constexpr const char* kBuildTime = __DATE__ " " __TIME__;

// ---------------------------------------------------------------------------
// 命令行
// ---------------------------------------------------------------------------

struct Options
{
    enum class Source
    {
        Camera,
        Image
    };

    Source source = Source::Camera;

    string value;                 // 图片路径
    string outDir = "output";
    int cameraIndex = 0;

    int votes = 4;                // 最近 5 帧里命中几帧才认定
    bool autoSave = false;        // 兼容旧参数（现在默认就是全自动保存）
    bool fast = false;            // 小图快跑（640），低配虚拟机用

    int resendSeconds = 3;        // 同一结论每隔几秒重发一次（持续识别，防漏收）
    bool fireOnce = true;         // 每种侧只发一次 F（--refire 可改成每次都发）

    string serialPort;            // 例如 /dev/ttyUSB0；空 = 不发串口
    int serialBaud = 115200;
    bool serialText = false;      // 发 "left"/"right" 而不是单字符 L / R
    bool ackNeeded = false;       // 等单片机回 ACK / PONG（默认只发不等）
    bool fireAfterSend = false;   // 发完 L/R 后自动再发 F（开火）

    bool showWindow = true;
    bool debug = false;           // 额外保存透视矫正后的标签图
    bool help = false;
    bool showVersion = false;     // --version / -v
};

void printUsage(const char* program)
{
    const string p = program;

    cout
        << "RM_HighScore   build " << kBuildId << "  (" << kBuildNote << ")\n"
        << "编译时间 " << kBuildTime << "\n"
        << "\n"
        << "用法:\n"
        << "  " << p << "                       用摄像头实时识别（默认方式）\n"
        << "  " << p << " --image tag.png        识别一张图片\n"
        << "\n"
        << "可选参数:\n"
        << "  --camera N      摄像头编号，默认 0\n"
        << "  --out DIR       输出目录，默认 output\n"
        << "  --votes N       连续一致多少帧才锁定，默认 4（最近 5 帧）\n"
        << "  --auto          兼容旧参数（现在默认就是全自动，不用按键）\n"
        << "  --fast          用 640 的分析尺寸跑，低配虚拟机更流畅\n"
        << "  --serial PORT   识别出结果后通过串口发给单片机，例如 /dev/ttyUSB0\n"
        << "  --baud N        串口波特率，默认 115200\n"
        << "  --serial-text   发 left / right 文本（默认发单字符 L / R）\n"
        << "  --resend N      同一结果每隔 N 秒重发一次，默认 3（持续识别不中断）\n"
        << "  --refire        每次重发都带开火 F（默认每种侧只发一次 F）\n"
        << "  --ack           等单片机回 ACK（默认只发不等，板子不回也照样工作）\n"
        << "  --fire          发完 L/R 后自动再发 F（开火）\n"
        << "  --no-window     不弹出窗口（没有图形界面时使用）\n"
        << "  --debug         额外保存透视矫正后的标签图（output/tag_warped.png）\n"
        << "  -v, --version   只打印构建编号然后退出\n"
        << "  -h, --help      显示本帮助\n"
        << "\n"
        << "摄像头模式（全自动，比赛时不需要任何按键）:\n"
        << "  一直识别；结论变化或每 --resend 秒自动保存并发送给单片机\n"
        << "  [ESC]/[q] 退出（仅调试）    [空格] 立刻补发一次（仅调试）\n"
        << "\n"
        << "输出:\n"
        << "  output/result.png   识别结果截图\n"
        << "  output/result.txt   第一行就是 high score:left / high score:right\n"
        << "\n"
        << "判定规则（与官方图例一致）:\n"
        << "  圆与方块颜色相同  -> 三角形尖角指向的一侧是高分区\n"
        << "  圆与方块颜色不同  -> 三角形尖角指向的反侧是高分区\n"
        << "\n"
        << "退出码:\n"
        << "  0 成功   2 参数错误   3 打不开图像/摄像头\n"
        << "  5 没识别出结果   6 写文件失败\n";
}

bool parseArgs(int argc, char** argv, Options& options, string& error)
{
    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];
        string inlineValue;
        bool hasInlineValue = false;

        const size_t equal = arg.find('=');

        if (arg.rfind("--", 0) == 0 && equal != string::npos)
        {
            inlineValue = arg.substr(equal + 1);
            arg = arg.substr(0, equal);
            hasInlineValue = true;
        }

        auto takeValue = [&](string& value) -> bool
        {
            if (hasInlineValue)
            {
                value = inlineValue;
                return true;
            }

            if (i + 1 >= argc)
            {
                return false;
            }

            value = argv[++i];
            return true;
        };

        if (arg == "-h" || arg == "--help")
        {
            options.help = true;
        }
        else if (arg == "-v" || arg == "--version")
        {
            options.showVersion = true;
        }
        else if (arg == "--image")
        {
            if (!takeValue(options.value))
            {
                error = "--image 需要一个图片路径";
                return false;
            }
            options.source = Options::Source::Image;
        }
        else if (arg == "--camera")
        {
            string value;

            if (!takeValue(value))
            {
                error = "--camera 需要一个编号";
                return false;
            }

            try
            {
                options.cameraIndex = stoi(value);
            }
            catch (const exception&)
            {
                error = "--camera 的取值不是整数：" + value;
                return false;
            }

            options.source = Options::Source::Camera;
        }
        else if (arg == "--out")
        {
            if (!takeValue(options.outDir))
            {
                error = "--out 需要一个目录路径";
                return false;
            }
        }
        else if (arg == "--votes")
        {
            string value;

            if (!takeValue(value))
            {
                error = "--votes 需要一个数字";
                return false;
            }

            try
            {
                options.votes = stoi(value);
            }
            catch (const exception&)
            {
                error = "--votes 的取值不是整数：" + value;
                return false;
            }

            options.votes = std::max(1, std::min(5, options.votes));
        }
        else if (arg == "--auto")
        {
            options.autoSave = true;
        }
        else if (arg == "--fast")
        {
            options.fast = true;
        }
        else if (arg == "--serial")
        {
            if (!takeValue(options.serialPort))
            {
                error = "--serial 需要一个串口设备路径";
                return false;
            }
        }
        else if (arg == "--baud")
        {
            string value;

            if (!takeValue(value))
            {
                error = "--baud 需要一个数字";
                return false;
            }

            try
            {
                options.serialBaud = stoi(value);
            }
            catch (const exception&)
            {
                error = "--baud 的取值不是整数：" + value;
                return false;
            }
        }
        else if (arg == "--serial-text")
        {
            options.serialText = true;
        }
        else if (arg == "--resend")
        {
            string value;

            if (!takeValue(value))
            {
                error = "--resend 需要一个秒数";
                return false;
            }

            try
            {
                options.resendSeconds = stoi(value);
            }
            catch (const exception&)
            {
                error = "--resend 的取值不是整数：" + value;
                return false;
            }

            options.resendSeconds = std::max(1, options.resendSeconds);
        }
        else if (arg == "--refire")
        {
            options.fireOnce = false;
        }
        else if (arg == "--ack")
        {
            options.ackNeeded = true;
        }
        else if (arg == "--fire")
        {
            options.fireAfterSend = true;
        }
        else if (arg == "--no-window")
        {
            options.showWindow = false;
        }
        else if (arg == "--debug")
        {
            options.debug = true;
        }
        else
        {
            error = "未知参数：" + arg;
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// 公共：输出目录、保存、打印
// ---------------------------------------------------------------------------

bool prepareOutput(const Options& options, fs::path& outDir)
{
    error_code ec;
    fs::create_directories(options.outDir, ec);

    if (ec)
    {
        cerr << "无法创建输出目录 " << options.outDir << "：" << ec.message() << endl;
        return false;
    }

    outDir = fs::path(options.outDir);
    return true;
}

bool saveAll(const Options& options, const fs::path& outDir,
             const cv::Mat& canvas, const TagResult& result)
{
    Visualizer visualizer;

    if (!visualizer.saveImage(canvas, (outDir / "result.png").string()))
    {
        return false;
    }

    if (!visualizer.saveText(result, (outDir / "result.txt").string()))
    {
        return false;
    }

    if (options.debug && !result.warped.empty())
    {
        visualizer.saveImage(result.warped, (outDir / "tag_warped.png").string());
    }

    return true;
}

// 时间戳，写日志用
string nowStamp()
{
    const time_t t = time(nullptr);
    tm localTime {};

#if defined(_WIN32)
    localtime_s(&localTime, &t);
#else
    localtime_r(&t, &localTime);
#endif

    char buffer[32] = {0};
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &localTime);

    return string(buffer);
}

// 每次发送都追加一行到 output/log.txt，赛后给裁判看也方便
void appendLog(const fs::path& outDir, const string& line)
{
    ofstream file((outDir / "log.txt").string(), ios::app);

    if (file.is_open())
    {
        file << line << '\n';
    }
}

void printResult(const TagResult& result)
{
    cout << "\n---------- 识别结果 ----------\n";
    cout << highScoreText(result.highSide) << "\n";
    cout << "  标签     : " << (result.tagFound ? "已找到" : "未找到") << "\n";
    cout << "  尖角朝向 : " << sideName(result.tipSide)
         << "（把握 " << result.tipMargin << "）\n";
    cout << "  圆形颜色 : " << colorName(result.circleColor) << "\n";
    cout << "  方块颜色 : " << colorName(result.squareColor) << "\n";
    cout << "  颜色关系 : " << (result.sameColor ? "相同（取尖角侧）" : "不同（取反侧）") << "\n";
    cout << "  说明     : " << result.reason << "\n";
    cout << "------------------------------\n\n";
}

// 开机探针：先发 P，确认单片机固件在线
static bool probeMcu(SerialLink& link)
{
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        if (!link.sendLine("P"))
        {
            return false;
        }

        if (link.readLine(400) == "PONG")
        {
            return true;
        }
    }

    return false;
}

// 发一条命令并等 ACK，最多重试 3 次（掉线/单片机复位都能救回来）
static bool sendWithAck(SerialLink& link, const string& command, const string& expectPrefix)
{
    for (int attempt = 1; attempt <= 3; ++attempt)
    {
        if (!link.sendLine(command))
        {
            cerr << "串口写入失败。" << endl;
            return false;
        }

        const string reply = link.readLine(600);

        if (reply.rfind(expectPrefix, 0) == 0)
        {
            return true;
        }

        cout << "第 " << attempt << " 次没收到 " << expectPrefix
             << (reply.empty() ? "（超时）" : "（收到：" + reply + "）")
             << "，重发……" << endl;

        this_thread::sleep_for(chrono::milliseconds(80));
    }

    return false;
}

// 把识别结果发给单片机：默认发 "L\n" / "R\n"（左/右），加 --serial-text 发 "left\n"/"right\n"
// 默认只发不等，单片机不回 ACK 也能用；加 --ack 才做 PONG / ACK 握手
void sendToMcu(const Options& options, const TagResult& result, bool allowFire = true)
{
    if (options.serialPort.empty() || !result.decided)
    {
        return;
    }

    SerialLink link;

    if (!link.open(options.serialPort, options.serialBaud))
    {
        cerr << "串口没打开，跳过发送（识别结果本身不受影响）。" << endl;
        return;
    }

    link.flush();

    const bool isLeft = (result.highSide == Side::Left);
    const string command = options.serialText ? sideName(result.highSide)
                                              : (isLeft ? "L" : "R");

    if (!options.ackNeeded)
    {
        if (!link.sendLine(command))
        {
            cerr << "串口写入失败：" << options.serialPort << endl;
            return;
        }

        cout << "串口已发送: " << command
             << "（" << options.serialPort << " @" << options.serialBaud << "）" << endl;
    }
    else
    {
        if (!probeMcu(link))
        {
            cerr << "\n没有收到单片机的 PONG —— 说明“PC → 板子 / 板子 → PC”有一条不通。按顺序查：\n"
                 << "  1) 板子上跑的是不是 RM_Link_Test 固件？上电后 PC13 LED 应约 2 秒闪一下。\n"
                 << "     不闪 = 没烧进去 / BOOT0 还在 1：拨回 0 再按一次 RESET。\n"
                 << "     烧录命令：stm32flash -w RM_Link_Test.hex -v -g 0x0 /dev/ttyUSB0（BOOT0=1 时）\n"
                 << "  2) 先看板子有没有主动打招呼：timeout 2 cat /dev/ttyUSB0\n"
                 << "     能出 BOOT:RM_LINK → 板子→PC 通，问题在 PC→板子（TXD 到 PA10）\n"
                 << "     出不来       → RXD（PA9）没接好 / 接反 / 没共地\n"
                 << "  3) CH340 与板子必须交叉并共地：TXD→PA10、RXD→PA9、GND→GND\n"
                 << "  4) CH340 跳线 3.3V，波特率 115200，串口没被 minicom 等占用\n"
                 << "  5) 仍不行就把 TXD/RXD 对调试一次（有的模块丝印是模块视角）\n" << endl;
            return;
        }

        cout << "单片机在线（PONG）" << endl;

        if (!sendWithAck(link, command, "ACK:"))
        {
            cerr << "单片机在线但不认命令：确认板上固件是 RM_Link_Test（应回 ACK:L / ACK:R）。" << endl;
            return;
        }

        cout << "已发送串口命令：" << command
             << "（高分区在" << (isLeft ? "左" : "右") << "）" << endl;
    }

    if (options.fireAfterSend && allowFire)
    {
        // 给单片机留出转向时间，再发开火
        this_thread::sleep_for(chrono::milliseconds(300));

        const bool ok = options.ackNeeded ? sendWithAck(link, "F", "ACK:")
                                          : link.sendLine("F");

        if (ok)
        {
            cout << "已发送开火命令：F" << endl;
        }
        else
        {
            cerr << "开火命令发送失败，检查单片机是否处于 ESTOP（发 S 后需要重新发 L/R）。" << endl;
        }
    }
}

// ---------------------------------------------------------------------------
// 图片模式
// ---------------------------------------------------------------------------

int runImage(const Options& options)
{
    ImageSource source;
    cv::Mat image;

    if (!source.loadImage(options.value, image))
    {
        return 3;
    }

    fs::path outDir;
    if (!prepareOutput(options, outDir))
    {
        return 6;
    }

    TagDetector::Params params;

    if (options.fast)
    {
        params.analysisMaxSide = 640;
    }

    TagDetector detector(params);
    const TagResult result = detector.detect(image);

    Visualizer visualizer;
    const cv::Mat canvas = visualizer.drawResult(image, result);

    printResult(result);

    if (!saveAll(options, outDir, canvas, result))
    {
        return 6;
    }

    sendToMcu(options, result);

    cout << "已输出：" << (outDir / "result.png").string()
         << " / " << (outDir / "result.txt").string() << "\n";

    if (options.showWindow)
    {
        try
        {
            cv::imshow("RM HighScore", canvas);
            cout << "按任意键关闭窗口……\n";
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
        catch (const cv::Exception& error)
        {
            cerr << "无法打开显示窗口（可能没有图形界面）：" << error.what() << "\n";
        }
    }

    return result.decided ? 0 : 5;
}

// ---------------------------------------------------------------------------
// 摄像头模式
//
// 单帧偶尔会被反光/抖动带偏，所以这里做"多帧投票"：
// 最近 5 帧里有 N 帧给出同一个结论才锁定，锁定后画面冻结，等裁判确认。
// ---------------------------------------------------------------------------

int runCamera(const Options& options)
{
    ImageSource source;

    if (!source.openCamera(options.cameraIndex))
    {
        return 3;
    }

    fs::path outDir;
    if (!prepareOutput(options, outDir))
    {
        return 6;
    }

    TagDetector::Params params;

    if (options.fast)
    {
        params.analysisMaxSide = 640;
    }

    TagDetector detector(params);
    Visualizer visualizer;

    cout << "自动模式：一直识别、一直发送，比赛时不需要任何按键。" << endl;
    cout << "  连续 " << options.votes << " 帧一致就发一次；同一结果每 "
         << options.resendSeconds << " 秒重发一次" << endl;
    cout << "  [ESC]/[q] 退出（仅调试）；[空格] 立刻补发一次（仅调试）" << endl;

    cv::Mat frame;

    std::deque<Side> history;
    TagResult lastGood;                 // 最近一次识别成功的完整结果
    Side lastSentSide = Side::Unknown;
    Side lastFiredSide = Side::Unknown;
    int commits = 0;

    double fps = 0.0;
    int frameCount = 0;
    auto lastTick = chrono::steady_clock::now();
    auto lastCommit = chrono::steady_clock::now() - chrono::hours(1);

    int readFailures = 0;

    while (true)
    {
        if (!source.read(frame))
        {
            // 摄像头偶尔掉帧：等一会儿重试，不要直接退出（比赛中间断掉就废了）
            ++readFailures;

            cerr << "读取摄像头失败（第 " << readFailures << " 次），0.2 秒后重试……" << endl;
            this_thread::sleep_for(chrono::milliseconds(200));

            if (readFailures % 30 == 0)
            {
                cerr << "尝试重新打开摄像头……" << endl;
                source.release();

                if (source.openCamera(options.cameraIndex))
                {
                    cerr << "摄像头已重新打开。" << endl;
                }
            }

            if (readFailures > 300)
            {
                cerr << "摄像头长时间不可用，退出。" << endl;
                break;
            }

            continue;
        }

        readFailures = 0;

        const TagResult result = detector.detect(frame);

        if (result.decided)
        {
            lastGood = result;
        }

        history.push_back(result.decided ? result.highSide : Side::Unknown);

        while (history.size() > 5)
        {
            history.pop_front();
        }

        const int leftVotes = static_cast<int>(std::count(history.begin(), history.end(), Side::Left));
        const int rightVotes = static_cast<int>(std::count(history.begin(), history.end(), Side::Right));

        Side voted = Side::Unknown;

        if (leftVotes >= options.votes && leftVotes > rightVotes)
        {
            voted = Side::Left;
        }
        else if (rightVotes >= options.votes && rightVotes > leftVotes)
        {
            voted = Side::Right;
        }

        const auto now = chrono::steady_clock::now();

        // 结论变化，或者距上次发送超过 resendSeconds，就再发一次
        const bool newSide = (voted != Side::Unknown && voted != lastSentSide);
        const bool dueRepeat = (voted != Side::Unknown &&
                                now - lastCommit >= chrono::seconds(options.resendSeconds));

        if (voted != Side::Unknown && (newSide || dueRepeat))
        {
            TagResult committed = result;
            committed.highSide = voted;
            committed.decided = true;
            committed.sameColor = result.sameColor;

            lastGood = committed;
            lastSentSide = voted;
            lastCommit = now;
            ++commits;

            const cv::Mat canvas = visualizer.drawResult(frame, committed);

            saveAll(options, outDir, canvas, committed);

            const bool allowFire = options.fireAfterSend &&
                                   (!options.fireOnce || voted != lastFiredSide);

            sendToMcu(options, committed, allowFire);

            if (allowFire)
            {
                lastFiredSide = voted;
            }

            cout << "[" << nowStamp() << "] 第 " << commits << " 次: "
                 << highScoreText(voted)
                 << "  (投票 " << max(leftVotes, rightVotes) << "/5)" << endl;

            appendLog(outDir,
                      nowStamp() + " " + sideName(voted) +
                      " votes=" + to_string(max(leftVotes, rightVotes)) + "/5" +
                      " commits=" + to_string(commits) +
                      " build=" + kBuildId);
        }

        // 显示：当前帧没识别到时，继续显示上一次的结论，画面不会闪回 searching
        TagResult display = result;

        if (!display.decided && lastGood.decided)
        {
            display.tagFound = false;
            display.decided = true;
            display.highSide = lastGood.highSide;
            display.reason = string("last: ") + sideName(lastGood.highSide) + "  (still scanning)";
        }

        cv::Mat shown = visualizer.drawResult(frame, display);

        ++frameCount;

        const double elapsed = chrono::duration<double>(now - lastTick).count();

        if (elapsed >= 0.5)
        {
            fps = static_cast<double>(frameCount) / elapsed;
            frameCount = 0;
            lastTick = now;
        }

        ostringstream info;
        info << "fps " << static_cast<int>(fps + 0.5)
             << "   votes " << max(leftVotes, rightVotes) << "/5"
             << "   sent " << commits
             << "   last " << (lastSentSide == Side::Unknown ? string("-") : sideName(lastSentSide));

        cv::putText(shown, info.str(), cv::Point(20, shown.rows - 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

        // 右上角常驻构建号，一眼确认跑的是新代码
        cv::putText(shown, kBuildId, cv::Point(shown.cols - 235, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        cv::imshow("RM HighScore", shown);

        const int key = cv::waitKey(1) & 0xFF;

        if (key == 27 || key == 'q')
        {
            break;
        }

        if ((key == ' ' || key == 13) && lastGood.decided)
        {
            // 调试用：立刻补发一次
            sendToMcu(options, lastGood, false);
            lastCommit = now;
        }
    }

    source.release();
    cv::destroyAllWindows();

    return 0;
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    const auto programStart = chrono::steady_clock::now();

    Options options;
    string argError;

    if (!parseArgs(argc, argv, options, argError))
    {
        cerr << "参数错误：" << argError << "\n\n";
        printUsage(argv[0]);
        return 2;
    }

    if (options.showVersion)
    {
        cout << kBuildId << "  (" << kBuildNote << ")\n";
        cout << "编译时间 " << kBuildTime << "\n";
        return 0;
    }

    if (options.help)
    {
        printUsage(argv[0]);
        return 0;
    }

    cout << "========================================================\n";
    cout << "   RM 识别高分区   |   high score: left / right\n";
    cout << "   build " << kBuildId << "  (" << kBuildNote << ")\n";
    cout << "   编译于 " << kBuildTime << "\n";
    cout << "========================================================\n";

    const int code = (options.source == Options::Source::Image)
        ? runImage(options)
        : runCamera(options);

    const double totalMs = chrono::duration<double, std::milli>(
        chrono::steady_clock::now() - programStart).count();

    cout << "\n总耗时：" << fixed << setprecision(1) << totalMs << " ms\n";

    return code;
}


