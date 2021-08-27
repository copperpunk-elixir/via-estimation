defmodule ViaEstimation.Matrix.UpdateFromGpsTest do
  use ExUnit.Case
  require Logger

  test "Predict Test" do
    config = [
      init_std_devs: [0.1, 0.1, 0.3, 0.1, 0.1, 0.3, 0.05],
      qpos_xy_std: 0.1,
      qpos_z_std: 0.05,
      qvel_xy_std: 0.05,
      qvel_z_std: 0.1,
      qyaw_std: 0.08,
      gpspos_xy_std: 0.715,
      gpspos_z_std: 2.05,
      gpsvel_xy_std: 0.088,
      gpsvel_z_std: 0.31,
      gpsyaw_std: 0.02,
      expected_imu_dt_s: 0.01,

      # Mahony
      imu_config: [
        imu_type: ViaEstimation.Imu.Mahony,
        imu_parameters: [
          kp: 0.1,
          ki: 0
        ]
      ]
    ]

    dt_accel_gyro = %{
      dt_s: 0.1,
      ax_mpss: 0.5,
      ay_mpss: 1.0,
      az_mpss: -9.8,
      gx_rps: 0.01,
      gy_rps: 0.02,
      gz_rps: -0.03
    }

    num_ops = 10

    heading_rad_init = ViaUtils.Math.deg2rad(15.2)
    heading_rad = ViaUtils.Math.deg2rad(16.0)

    ekf_hc = ViaEstimation.Ekf.SevenState.new(config)

    start_time_hc = :erlang.monotonic_time(:nanosecond)

    ekf_hc =
      Enum.reduce(1..num_ops, ekf_hc, fn _x, ekf ->
        ViaEstimation.Ekf.SevenState.predict(ekf, dt_accel_gyro)
      end)

    ViaEstimation.Ekf.SevenState.update_from_heading(ekf_hc, heading_rad_init)
    |> ViaEstimation.Ekf.SevenState.update_from_heading(heading_rad)

    end_time_hc = :erlang.monotonic_time(:nanosecond)

    IO.puts(
      "dt via: #{ViaUtils.Format.eftb((end_time_hc - start_time_hc) * 1.0e-6 / num_ops, 3)} ms"
    )
  end
end
