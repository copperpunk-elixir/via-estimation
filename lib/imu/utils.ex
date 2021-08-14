defmodule ViaEstimation.Imu.Utils do
  @spec rotate_yaw_rad(struct(), float()) :: struct()
  def rotate_yaw_rad(imu, delta_yaw_rad) do
    yaw = imu.yaw_rad + delta_yaw_rad

    %{imu | yaw_rad: yaw}
    |> reset_quat_to_attitude()
  end

  @spec reset_quat_to_attitude(struct()) :: struct()
  def reset_quat_to_attitude(imu) do
    cr = :math.cos(imu.roll_rad * 0.5)
    sr = :math.sin(imu.roll_rad * 0.5)
    cp = :math.cos(imu.pitch_rad * 0.5)
    sp = :math.sin(imu.pitch_rad * 0.5)
    cy = :math.cos(imu.yaw_rad * 0.5)
    sy = :math.sin(imu.yaw_rad * 0.5)
    crcp = cr * cp
    spsy = sp * sy
    spcy = sp * cy
    srcp = sr * cp

    q0 = crcp * cy + sr * spsy
    q1 = srcp * cy - cr * spsy
    q2 = cr * spcy + srcp * sy
    q3 = crcp * sy - sr * spcy
    %{imu | quat: {q0, q1, q2, q3}}
  end

  @spec rpy_to_string(struct(), integer()) :: binary()
  def rpy_to_string(imu, decimals) do
    rpy =
      Enum.map([imu.roll_rad, imu.pitch_rad, imu.yaw_rad], fn x ->
        ViaUtils.Math.rad2deg(x)
      end)

    ViaUtils.Format.eftb_list(rpy, decimals)
  end
end