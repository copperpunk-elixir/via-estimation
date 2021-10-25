defmodule ViaEstimation.Ekf.Agl do
  require Logger

  # If this looks like I don't know what I'm doing, that's because it's true.
  defstruct roll_rad: 0,
            pitch_rad: 0,
            z_m: 0,
            q33: 0,
            p00: 0,
            p11: 0,
            p22: 0,
            p33: 0,
            r: 0,
            time_prev_us: nil,
            roll_max_rad: 0,
            pitch_max_rad: 0

  @spec new(list()) :: struct()
  def new(config) do
    q_att_sq = Keyword.fetch!(config, :q_att_sq)
    q_zdot_sq = Keyword.fetch!(config, :q_zdot_sq)
    q_z_sq = Keyword.fetch!(config, :q_z_sq)
    r_range_sq = Keyword.fetch!(config, :r_range_sq)
    roll_max_rad = Keyword.fetch!(config, :roll_max_rad)
    pitch_max_rad = Keyword.fetch!(config, :pitch_max_rad)

    %ViaEstimation.Ekf.Agl{
      q33: q_z_sq,
      p00: q_att_sq,
      p11: q_att_sq,
      p22: q_zdot_sq,
      p33: q_z_sq,
      r: r_range_sq,
      roll_max_rad: roll_max_rad,
      pitch_max_rad: pitch_max_rad
    }
  end

  @spec reset(struct(), float()) :: struct()
  def reset(ekf, z_m) do
    %{p00: p00, p22: p22, q33: q33, r: r} = ekf

    config = [
      q_att_sq: p00,
      q_zdot_sq: p22,
      q_z_sq: q33,
      r_range_sq: r
    ]

    ekf = ViaEstimation.Ekf.Agl.new(config)
    %{ekf | z_m: z_m}
  end

  @spec predict(struct(), float(), float(), float()) :: struct()
  def predict(ekf, roll_rad, pitch_rad, zdot_mps) do
    current_time_us = :os.system_time(:microsecond)

    %{time_prev_us: time_prev_us, z_m: z_m, p22: p22, p33: p33, q33: q33} = ekf

    dt_s = if is_nil(time_prev_us), do: 0, else: (current_time_us - time_prev_us) * 1.0e-6
    z_m = z_m + zdot_mps * dt_s
    # Logger.debug("zdot/zprev/z/dt: #{zdot}/#{ekf.z}/#{z}/#{dt_s}")
    p33 = p33 + p22 * dt_s * dt_s + q33

    %{
      ekf
      | roll_rad: roll_rad,
        pitch_rad: pitch_rad,
        z_m: z_m,
        p33: p33,
        time_prev_us: current_time_us
    }
  end

  @spec update_from_range(struct(), float()) :: struct()
  def update_from_range(ekf, range_meas_m) do
    %{
      roll_rad: roll_rad,
      pitch_rad: pitch_rad,
      roll_max_rad: roll_max_rad,
      pitch_max_rad: pitch_max_rad,
      z_m: z_m,
      r: r,
      p00: p00,
      p11: p11,
      p33: p33
    } = ekf

    roll_rad = roll_rad
    pitch_rad = pitch_rad

    if abs(roll_rad) > roll_max_rad or abs(pitch_rad) > pitch_max_rad do
      ekf
    else
      z_sq = z_m * z_m
      sinphi = :math.sin(roll_rad)
      sinphi_sq = sinphi * sinphi
      cosphi = :math.cos(roll_rad)
      cosphi_sq = cosphi * cosphi
      sintheta = :math.sin(pitch_rad)
      sintheta_sq = sintheta * sintheta
      costheta = :math.cos(pitch_rad)
      costheta_sq = costheta * costheta

      s =
        r + p33 / (cosphi_sq * costheta_sq) +
          p00 * (sinphi_sq * z_sq) / (cosphi_sq * cosphi_sq * costheta_sq) +
          p11 * (sintheta_sq * z_sq) / (cosphi_sq * costheta_sq * costheta_sq)

      s = if s == 0, do: 0.0, else: 1 / s
      dz = range_meas_m - z_m / (cosphi * costheta)
      z_m = z_m + s * dz * p33 / (cosphi_sq * costheta_sq)
      p33 = p33 * (1.0 - p33 * s / (cosphi_sq * costheta_sq))
      %{ekf | z_m: z_m, p33: p33}
    end
  end

  @spec agl_m(struct()) :: float()
  def agl_m(ekf) do
    ekf.z_m
  end
end
