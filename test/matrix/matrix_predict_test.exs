defmodule ViaEstimation.Matrix.PredictTest do
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
    ekf_matrex = ViaEstimation.Ekf.SevenState.new(config)

    start_time_matrex = :erlang.monotonic_time(:nanosecond)

    ekf_matrex =
      Enum.reduce(1..num_ops, ekf_matrex, fn _x, ekf ->
        ViaEstimation.Ekf.SevenState.predict(ekf, dt_accel_gyro)
      end)

    end_time_matrex = :erlang.monotonic_time(:nanosecond)
    IO.puts("matrex state: #{inspect(ekf_matrex.ekf_state)}")
    IO.puts("matrex cov: #{inspect(ekf_matrex.ekf_cov)}")

    ekf_hardcode = ViaEstimation.Ekf.SevenState.new(config)
    ekf_state_hc = ekf_hardcode.ekf_state |> Matrex.to_list() |> List.to_tuple()
    ekf_cov_hc = ekf_hardcode.ekf_cov |> Matrex.to_list() |> List.to_tuple()
    q_ekf_hc = ekf_hardcode.q_ekf |> Matrex.diagonal() |> Matrex.to_list() |> List.to_tuple()
    ekf_hardcode = %{ekf_hardcode | ekf_state: ekf_state_hc, ekf_cov: ekf_cov_hc, q_ekf: q_ekf_hc}
    start_time_hc = :erlang.monotonic_time(:nanosecond)

    ekf_hc =
      Enum.reduce(1..num_ops, ekf_hardcode, fn _x, ekf ->
          ViaEstimation.Matrix.predict(ekf, dt_accel_gyro)
      end)

    end_time_hc = :erlang.monotonic_time(:nanosecond)
    ekf_state_hc_matrex = ViaEstimation.Matrix.tuple_to_matrex(ekf_hc.ekf_state, 7, 1)
    ekf_cov_hc_matrex = ViaEstimation.Matrix.tuple_to_matrex(ekf_hc.ekf_cov, 7, 7)
    IO.puts("hc state: #{inspect(ekf_state_hc_matrex)}")
    IO.puts("hc cov: #{inspect(ekf_cov_hc_matrex)}")

    IO.puts(
      "dt matrex: #{ViaUtils.Format.eftb((end_time_matrex - start_time_matrex) * 1.0e-6 / num_ops, 3)} ms"
    )

    IO.puts(
      "dt via: #{ViaUtils.Format.eftb((end_time_hc - start_time_hc) * 1.0e-6 / num_ops, 3)} ms"
    )

    Enum.each(1..7, fn i ->
      Enum.each(1..1, fn j ->
        assert_in_delta(
          Matrex.at(ekf_matrex.ekf_state, i, j),
          Matrex.at(ekf_state_hc_matrex, i, j),
          0.0001
        )
      end)
    end)

    Enum.each(1..7, fn i ->
      Enum.each(1..7, fn j ->
        assert_in_delta(
          Matrex.at(ekf_matrex.ekf_cov, i, j),
          Matrex.at(ekf_cov_hc_matrex, i, j),
          0.0001
        )
      end)
    end)
  end
end
