defmodule ViaEstimation.Ekf.SevenState.FakeUpdateTest do
  use ExUnit.Case
  require Logger

  setup do
    RingLogger.attach()
    {:ok, []}
  end

  test "Run full EKF cycle" do
    ins_kf_config = [
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
      imu_config: [
        imu_type: ViaEstimation.Imu.Mahony,
        imu_parameters: [
          kp: 0.1,
          ki: 0
        ]
      ]
    ]

    ekf = ViaEstimation.Ekf.SevenState.new(ins_kf_config)
    position = ViaUtils.Location.new_location_input_degrees(42, -120, 123)
    velocity = %{north_mps: 1.0, east_mps: 0 * 2.0, down_mps: 0 * -3.0}

    ekf =
      ViaEstimation.Ekf.SevenState.update_from_gps(ekf, position, velocity)
      |> ViaEstimation.Ekf.SevenState.update_from_heading(0.1)

    dt_accel_gyro = %{
      dt_s: 0.05,
      ax_mpss: 1.0,
      ay_mpss: 0,
      az_mpss: 0,
      gx_rps: 0.1,
      gy_rps: 0,
      gz_rps: 0
    }

    ekf =
      Enum.reduce(1..2000, ekf, fn _x, acc ->
        # start_time = :erlang.monotonic_time(:nanosecond)
        acc = ViaEstimation.Ekf.SevenState.predict(acc, dt_accel_gyro)
        # end_time = :erlang.monotonic_time(:nanosecond)
        # IO.puts("predict dt: #{(end_time - start_time) * 1.0e-6}")
        acc
      end)

    IO.inspect(ekf)

    IO.puts(
      "position: #{ViaUtils.Location.to_string(ViaEstimation.Ekf.SevenState.position_rrm(ekf))}"
    )

    {position, velocity} = ViaEstimation.Ekf.SevenState.position_rrm_velocity_mps(ekf)
    IO.puts("position: #{ViaUtils.Location.to_string(position)}")
    IO.puts("velocity: #{ViaUtils.Format.eftb_map(velocity, 2)}")
  end
end
