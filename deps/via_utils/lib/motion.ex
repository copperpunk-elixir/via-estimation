defmodule ViaUtils.Motion do
  require Logger
  require ViaUtils.Constants, as: VC

  @moduledoc """
  Documentation for `UtilsMotion`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> UtilsMotion.hello()
      :world

  """

  # Convert North/East velocity to Speed/Course
  @spec get_speed_course_for_velocity(number(), number(), number(), number()) :: tuple()
  def get_speed_course_for_velocity(v_north, v_east, min_speed_for_course, yaw) do
    speed = ViaUtils.Math.hypot(v_north, v_east)

    course =
      if speed >= min_speed_for_course do
        :math.atan2(v_east, v_north)
        |> ViaUtils.Math.constrain_angle_to_compass()
      else
        # Logger.info("too slow: #{speed}")
        yaw
      end

    {speed, course}
  end

  @spec adjust_velocity_for_min_speed(map(), number(), number()) :: map()
  def adjust_velocity_for_min_speed(velocity, min_speed_for_course, yaw) do
    speed = ViaUtils.Math.hypot(velocity.north, velocity.east)

    if speed >= min_speed_for_course do
      velocity
    else
      %{velocity | north: speed * :math.cos(yaw), east: speed * :math.sin(yaw)}
    end
  end

  # Turn correctly left or right using delta Yaw/Course
  @spec turn_left_or_right_for_correction(number()) :: number()
  def turn_left_or_right_for_correction(correction) do
    cond do
      correction < -:math.pi() -> correction + 2.0 * :math.pi()
      correction > :math.pi() -> correction - 2.0 * :math.pi()
      true -> correction
    end
  end

  @spec attitude_to_accel_rad(map()) :: map()
  def attitude_to_accel_rad(attitude) do
    cos_theta = :math.cos(attitude.pitch_rad)

    ax = :math.sin(attitude.pitch_rad)
    ay = -:math.sin(attitude.roll_rad) * cos_theta
    az = -:math.cos(attitude.roll_rad) * cos_theta

    %{
      x: ax * VC.gravity(),
      y: ay * VC.gravity(),
      z: az * VC.gravity()
    }
  end

  @spec inertial_to_body_euler_rad(map(), tuple()) :: tuple()
  def inertial_to_body_euler_rad(attitude, vector) do
    cosphi = :math.cos(attitude.roll_rad)
    sinphi = :math.sin(attitude.roll_rad)
    costheta = :math.cos(attitude.pitch_rad)
    sintheta = :math.sin(attitude.pitch_rad)
    cospsi = :math.cos(attitude.yaw_rad)
    sinpsi = :math.sin(attitude.yaw_rad)

    {vx, vy, vz} = vector

    bx = costheta * cospsi * vx + costheta * sinpsi * vy - sintheta * vz

    by =
      (-cosphi * sinpsi + sinphi * sintheta * cospsi) * vx +
        (cosphi * cospsi + sinphi * sintheta * sinpsi) * vy + sinphi * costheta * vz

    bz =
      (sinphi * sinpsi + cosphi * sintheta * cospsi) * vx -
        (sinphi * cospsi + cosphi * sintheta * sinpsi) * vy + cosphi * costheta * vz

    {bx, by, bz}
  end

  @spec inertial_to_body_euler_deg(map(), tuple()) :: tuple()
  def inertial_to_body_euler_deg(attitude, vector) do
    attitude_rad = %{
      roll_rad: ViaUtils.Math.deg2rad(attitude.roll_deg),
      pitch_rad: ViaUtils.Math.deg2rad(attitude.pitch_deg),
      yaw_rad: ViaUtils.Math.deg2rad(attitude.yaw_deg)
    }
    inertial_to_body_euler_rad(attitude_rad, vector)
  end

  @spec quaternion_to_euler(float(), float(), float(), float()) :: map()
  def quaternion_to_euler(q0, q1, q2, q3) do
    roll = :math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = :math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = :math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    yaw =
      cond do
        yaw < 0 -> yaw + VC.two_pi()
        yaw >= VC.two_pi() -> yaw - VC.two_pi()
        true -> yaw
      end

    %{
      roll: roll,
      pitch: pitch,
      yaw: yaw
    }
  end
end
