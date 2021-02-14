#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"
#include "logger.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    vector<Eigen::Vector4d> estimations;
    vector<Eigen::Vector4d> ground_truth;

    h.onMessage([&fusionEKF, &estimations, &ground_truth]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data));

            if (s != "") {
                BOOST_LOG_TRIVIAL(info) << "Received sensor update";
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    string sensor_measurement = j[1]["sensor_measurement"];

                    MeasurementPackage meas_package;
                    std::istringstream iss(sensor_measurement);

                    long long timestamp;

                    // reads first element from the current line
                    string sensor_type;
                    iss >> sensor_type;

                    if (sensor_type.compare("L") == 0) {
                        meas_package.sensor_type_ = MeasurementPackage::LASER;
                        meas_package.raw_measurements_ = VectorXd(2);
                        float px;
                        float py;
                        iss >> px;
                        iss >> py;
                        meas_package.raw_measurements_ << px, py;
                        iss >> timestamp;
                        meas_package.timestamp_ = timestamp;
                    } else if (sensor_type.compare("R") == 0) {
                        meas_package.sensor_type_ = MeasurementPackage::RADAR;
                        meas_package.raw_measurements_ = VectorXd(3);
                        float ro;
                        float theta;
                        float ro_dot;
                        iss >> ro;
                        iss >> theta;
                        iss >> ro_dot;
                        meas_package.raw_measurements_ << ro,theta, ro_dot;
                        iss >> timestamp;
                        meas_package.timestamp_ = timestamp;
                    }

                    Eigen::Vector4d gt_values(4);
                    iss >> gt_values(0);
                    iss >> gt_values(1);
                    iss >> gt_values(2);
                    iss >> gt_values(3);
                    ground_truth.push_back(gt_values);

                    // Call ProcessMeasurement(meas_package) for Kalman filter
                    fusionEKF.ProcessMeasurement(meas_package);

                    // Push the current estimated x,y position from the Kalman filter's state vector
                    estimations.push_back(fusionEKF.ekf_.x_);

                    Eigen::Vector4d RMSE = Tools::CalculateRMSE(estimations, ground_truth);

                    // Build up return message
                    json msgJson;
                    msgJson["estimate_x"] = fusionEKF.ekf_.x_[0];
                    msgJson["estimate_y"] = fusionEKF.ekf_.x_[1];
                    msgJson["rmse_x"] =  RMSE(0);
                    msgJson["rmse_y"] =  RMSE(1);
                    msgJson["rmse_vx"] = RMSE(2);
                    msgJson["rmse_vy"] = RMSE(3);
                    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    BOOST_LOG_TRIVIAL(info) << msgJson;
                }  // end "telemetry" if

            } else {
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if

    }); // end h.onMessage

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        BOOST_LOG_TRIVIAL(info) << "Connected";
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
        ws.close();
        BOOST_LOG_TRIVIAL(info) << "Disconnected";
    });

    int port = 4567;
    if (h.listen(port)) {
        BOOST_LOG_TRIVIAL(info) << "Listening to port " << port;
    } else {
        BOOST_LOG_TRIVIAL(error) << "Failed to listen to port";
        return -1;
    }

    h.run();
}