#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "ukf_cfg.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPSI (0.001)

/* Private Debug Functions */
#ifdef DEBUG_GENERAL

static void PrintMatrix(string str, MatrixXd prnt)
{
    /* IOFormat inspired from https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html */
    const Eigen::IOFormat fmt(4, 0, ",\t", "", "\n\t[", "]");

    cout << str << prnt.format(fmt) << endl;
    cout << endl;
}

static void PrintVector(string str, VectorXd prnt)
{
    /* IOFormat inspired from https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html */
    const Eigen::IOFormat fmt(4, 0, ",\t", "", "\n\t[", "]");

    cout << str << prnt.format(fmt) << endl;
    cout << endl;
}
#endif

/* Private Functions */
static void calculateWeights(VectorXd & __out)
{
    double temp = (LAMDA + AUG_DCNT);

    __out.fill( ((0.5) / temp) );
    __out(0U) = (LAMDA / temp);
}

static void calculateMeasCovarMatrix(MatrixXd & __R_Lidar, MatrixXd & __R_Radar)
{
    __R_Lidar(0U, 0U) = ( LIDAR_STD_PX * LIDAR_STD_PX );
    __R_Lidar(1U, 1U) = ( LIDAR_STD_PY * LIDAR_STD_PY );

    __R_Radar(0U, 0U) = ( RADAR_STD_RHO * RADAR_STD_RHO );
    __R_Radar(1U, 1U) = ( RADAR_STD_PHI * RADAR_STD_PHI );
    __R_Radar(2U, 2U) = ( RADAR_STD_RHODOT * RADAR_STD_RHODOT );
}

/* Public Functions */
/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    /* Allocate Internal DS */
    x_         = VectorXd(STATE_DCNT);
    Xsig_pred_ = MatrixXd(STATE_DCNT, SIG_PTS_CNT);
    weights_   = VectorXd(SIG_PTS_CNT);

    R_lidar_   = MatrixXd(LID_DCNT, LID_DCNT);
    R_radar_   = MatrixXd(RAD_DCNT, RAD_DCNT);

    /* Initialize Internal DS values */
    x_.fill(0.0);
    /* TODO: Revise P initial values */
    P_         = MatrixXd::Identity(STATE_DCNT, STATE_DCNT);
    Xsig_pred_.fill(0.0);
    weights_.fill(0.0);

    R_lidar_.fill(0.0);
    R_radar_.fill(0.0);

    time_us_ = 0U;
    is_initialized_ = false;

    calculateWeights(weights_);
    calculateMeasCovarMatrix(R_lidar_, R_radar_);

    #ifdef DEBUG_INITPARAM
    PrintVector("State ", x_);
    PrintMatrix("Process Covar ", P_);
    PrintMatrix("SigmaPrediction ", Xsig_pred_);
    PrintVector("Weights ", weights_);
    PrintMatrix("R_Lidar ", R_lidar_);
    PrintMatrix("R_Radar ", R_radar_);
    #endif
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    if(false == is_initialized_) /* Process first measurements */
    {
        if(meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            double Px = meas_package.raw_measurements_[0];
            double Py = meas_package.raw_measurements_[1];

            x_ << Px, Py, 0U, 0U, 0U;

            #ifdef DEBUG_FIRSTMEAS
            cout << "Initialization done using LIDAR measurement." << endl;
            #endif
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            /* TODO: Correct Velocity Initialization */
            double Range     = meas_package.raw_measurements_[0];
            double Bearing   = meas_package.raw_measurements_[1];
            /* double RangeRate = meas_package.raw_measurements_[2]; */

            double Px = Range * cos(Bearing);
            /* double Vx = RangeRate * cos(Bearing); */
            if(Px < EPSI)
            {
                Px = EPSI;
            }

            double Py = Range * sin(Bearing);
            /* double Vy = RangeRate * sin(Bearing); */
            if(Py < EPSI)
            {
                Py = EPSI;
            }

            x_ << Px, Py, 0, 0, 0;

            #ifdef DEBUG_FIRSTMEAS
            cout << "Initialization done using RADAR measurement." << endl;
            #endif
        }

        time_us_ = meas_package.timestamp_ ;
        is_initialized_ = true;

        #ifdef DEBUG_FIRSTMEAS
        PrintVector("Initalized State ", x_);
        cout << "------------------------------DTP0001------------------------------" << endl;
        #endif
    }
    else /* Process later measurements */
    {
        /*****************************************************************************
         *  Prediction
         ****************************************************************************/

        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; /* dt - expressed in seconds */
        time_us_ = meas_package.timestamp_;

        Prediction(dt);

        #ifdef DEBUG_PREDICTION
        cout << "------------------------------DTP0002------------------------------" << endl;
        cout << "delta time = " << dt << endl;
        #endif

        /*****************************************************************************
         *  Update
         ****************************************************************************/

        if(meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            #ifdef ENABLE_LASER
            #endif
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            #ifdef ENABLE_RADAR
            #endif
        }
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
}

/* Getter and Setter function */
VectorXd UKF::get_StateEstimate(void)
{
    VectorXd ret = VectorXd(OUT_DCNT);

    double Px  = x_(0U);
    double Py  = x_(1U);
    double V   = x_(2U);
    double Psi = x_(3U);

    ret(0U) = Px;
    ret(1U) = Py;
    ret(2U) = V * cos(Psi); /* Vx */
    ret(3U) = V * sin(Psi); /* Vy */
    return ret;
}