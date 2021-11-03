defmodule ViaEstimation.Imu.Utils do
  require ViaUtils.Shared.ValueNames, as: SVN

  @spec rotate_yaw_rad(struct(), float()) :: struct()
  def rotate_yaw_rad(imu, delta_yaw_rad) do
    %{yaw_rad: yaw_rad} = imu
    yaw = yaw_rad + delta_yaw_rad

    %{imu | yaw_rad: yaw}
    |> reset_quat_to_attitude()
  end

  @spec reset_quat_to_attitude(struct()) :: struct()
  def reset_quat_to_attitude(imu) do
    %{roll_rad: roll_rad, pitch_rad: pitch_rad, yaw_rad: yaw_rad} = imu
    cr = :math.cos(roll_rad * 0.5)
    sr = :math.sin(roll_rad * 0.5)
    cp = :math.cos(pitch_rad * 0.5)
    sp = :math.sin(pitch_rad * 0.5)
    cy = :math.cos(yaw_rad * 0.5)
    sy = :math.sin(yaw_rad * 0.5)
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
end
