#ifndef UKF_CFG_H
#define UKF_CFG_H

/* Debug Switches */
#define DEBUG_GENERAL

#ifdef DEBUG_GENERAL
// #define DEBUG_INITPARAM
// #define DEBUG_FIRSTMEAS
#define DEBUG_PREDICTION
#endif

/* Process Model Dimensions */
#define NOISE_DCNT (2U) /* Noise dimensions count                 */
#define LID_DCNT   (2U) /* Lidar measurement dimensions count     */
#define RAD_DCNT   (3U) /* Radar measurement dimensions count     */
#define STATE_DCNT (5U) /* State model dimensions count           */
#define AUG_DCNT   (7U) /* Augmented state model dimensions count */
#define OUT_DCNT   (4U) /* Output state model dimensions count    */

#define LAMDA_C(NAUG) (double)(3.0 - NAUG)
#define LAMDA         (double)(LAMDA_C(AUG_DCNT))

#define SIG_PTS_CNT_C(NAUG) ((2U * NAUG) + 1U)
#define SIG_PTS_CNT SIG_PTS_CNT_C(AUG_DCNT)

/* Swtich to enable/disable measurement updates of RADAR or LIDAR. */
#if ( defined (ENABLE_RADAR) || defined (ENABLE_LASER) )
    #error ERROR:(001) Internal switches configured outside of the UKF class.
#endif

#define ENABLE_RADAR
#define ENABLE_LASER

#if ( !defined (ENABLE_RADAR) && !defined (ENABLE_LASER) )
    #error ERROR:(002) UKF configuration inconsistent.
#endif

/* Process Model Noise Parameters */
#define PROCESS_LONG_ACC (30.0) /* Process noise standard deviation longitudinal acceleration in m/s^2 */
#define PROCESS_YAW_ACC  (30.0) /* Process noise standard deviation yaw acceleration in rad/s^2 */

/* Sensor defined standard deviation. */
/* DO NOT MODIFY BELOW THIS LINE TILL CLOSE TAG */
#define LIDAR_STD_PX     (0.15)
#define LIDAR_STD_PY     (0.15)

#define RADAR_STD_RHO    (0.3)
#define RADAR_STD_PHI    (0.03)
#define RADAR_STD_RHODOT (0.3)
/* DO NOT MODIFY ABOVE THIS LINE TILL OPEN TAG */

#endif /* UKF_CFG_H */
