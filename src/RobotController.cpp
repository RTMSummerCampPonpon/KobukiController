// -*- C++ -*-
/*!
 * @file  RobotController.cpp
 * @brief Robot Controller
 * @date $Date$
 *
 * @author 宮本　信彦　n-miyamoto@aist.go.jp
 * 産業技術総合研究所　ロボットイノベーション研究センター
 * ロボットソフトウエアプラットフォーム研究チーム
 *
 * $Id$
 */

#include "RobotController.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

bool isMoving = false;       // 動作中フラグ
double moveDuration = 10.0;   // 動作時間[s]
double elapsedTime = 0.0;    // 経過時間[s]
double period = 0.01;        // onExecute周期[s]（RTC周期に合わせる）
double zThreshold = 1;  // zの下限値
double xThreshold = 50;


// Module specification
// <rtc-template block="module_spec">
static const char* robotcontroller_spec[] =
  {
    "implementation_id", "RobotController",
    "type_name",         "RobotController",
    "description",       "Robot Controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.speed_x", "0.0",
    "conf.default.speed_r", "0.0",
    "conf.default.stop_d", "30",

    // Widget
    "conf.__widget__.speed_x", "slider.0.01",
    "conf.__widget__.speed_r", "slider.0.01",
    "conf.__widget__.stop_d", "text",
    // Constraints
    "conf.__constraints__.speed_x", "-1.5<x<1.5",
    "conf.__constraints__.speed_r", "-2.0<x<2.0",

    "conf.__type__.speed_x", "double",
    "conf.__type__.speed_r", "double",
    "conf.__type__.stop_d", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RobotController::RobotController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_currentPoseIn("currentPose", m_currentPose),
	m_coodIn("cood", m_cood),
    m_outOut("out", m_out),
    m_cmplOut("cmpl", m_cmpl)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RobotController::~RobotController()
{
}



RTC::ReturnCode_t RobotController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  addInPort("cood", m_coodIn);
  addInPort("currentPose", m_currentPoseIn);
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  addOutPort("cmpl", m_cmplOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("speed_x", m_speed_x, "0.0");
  bindParameter("speed_r", m_speed_r, "0.0");
  bindParameter("stop_d", m_stop_d, "300");

  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RobotController::onActivated(RTC::UniqueId ec_id)
{
	/*
	//コンフィギュレーションパラメータ初期化
	coil::Properties prop = m_configsets.getActiveConfigurationSet();
	prop.setProperty("speed_x", "0.0");
	prop.setProperty("speed_r", "0.0");
	m_configsets.setConfigurationSetValues(prop);
	m_configsets.activateConfigurationSet("default");
	*/

    dep = 0.0;
    x_px = 0.0;
    y_px = 0.0;
    //target_x_px = 640;
    //target_dep = 0;

	//センサ値初期化
	for (int i = 0; i < 4; i++)
	{
		sensor_data[i] = 0;
	}

  return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotController::onDeactivated(RTC::UniqueId ec_id)
{
	//ロボットを停止する
	m_out.data.vx = 0;
	m_out.data.va = 0;
	m_outOut.write();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotController::onExecute(RTC::UniqueId ec_id)
{
    /*
	//入力データの存在確認
	if (m_inIn.isNew())
	{
		//入力データ読み込み
		m_inIn.read();
		//この時点で入力データがm_inに格納される
		for (int i = 0; i < m_in.data.length(); i++)
		{
			//入力データを別変数に格納
			if (i < 4)
			{
				sensor_data[i] = m_in.data[i];
			}
		}
	}

	//前進するときのみ停止するかを判定
	if (m_speed_x > 0)
	{
		for (int i = 0; i < 4; i++)
		{
			//センサ値が設定値以上か判定
			if (sensor_data[i] > m_stop_d)
			{
				//センサ値が設定値以上の場合は停止
				m_out.data.vx = 0;
				m_out.data.va = 0;
				m_outOut.write();
				return RTC::RTC_OK;
			}
		}
	}
    */
	/*
	//設定値以上の値のセンサが無い場合はコンフィギュレーションパラメータの値で操作
	m_out.data.vx = m_speed_x;
	m_out.data.va = m_speed_r;
	m_outOut.write();
	*/
    /*
    if (m_currentPoseIn.isNew()) {
        while (!m_currentPoseIn.isEmpty()) m_currentPoseIn.read();
    }
    */


    if (m_coodIn.isNew()) {
        m_coodIn.read();
        x_px = m_cood.data[0];
        y_px = m_cood.data[1];
        dep = m_cood.data[2];
    }

    // dep がしきい値未満なら常に停止
    if (dep < zThreshold) {
        isMoving = false;
        m_out.data.vx = 0.0;
        m_out.data.va = 0.0;
        m_outOut.write();
        return RTC::RTC_OK;
    }

    // dep >= しきい値の場合は動作制御
    if (!isMoving) {
        // 新しい動作開始
        isMoving = true;
        elapsedTime = 0.0;

        // 動作開始 → 完了信号リセット
        m_cmpl.data = false;

        // 開始時の値を保持
        target_x_px = x_px;
        target_dep = dep;
    }

    // 動作中の処理
    if (isMoving) {
        float centerX = 640;

        if ((centerX - x_px) > xThreshold) {
            // 左に回転
            m_out.data.vx = 0;
            m_out.data.va = 0.5;
        }
        else if ((centerX - x_px) < -xThreshold) {
            // 右に回転
            m_out.data.vx = 0;
            m_out.data.va = -0.5;
        }
        else {
            // 中央付近なら前進
            m_out.data.vx = target_dep / moveDuration;
            m_out.data.va = 0;
        }
        m_outOut.write();

        elapsedTime += period;

        if (elapsedTime >= moveDuration) {
            // 1サイクル動作終了
            isMoving = false;

            // dep がまだしきい値以上なら次ループで再度開始される
            m_out.data.vx = 0.0;
            m_out.data.va = 0.0;
            m_outOut.write();

            // 完了信号を出力
            m_cmpl.data = true;
            m_cmplOut.write();
        }
    }




  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RobotControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(robotcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<RobotController>,
                             RTC::Delete<RobotController>);
  }
  
};


