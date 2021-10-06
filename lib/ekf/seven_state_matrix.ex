defmodule ViaEstimation.Ekf.SevenState do
  require Logger
  require ViaUtils.Constants, as: VC
  require ViaUtils.Shared.ValueNames, as: SVN

  defstruct ekf_state: nil,
            ekf_cov: nil,
            r_gps: nil,
            r_heading: nil,
            q_ekf: nil,
            imu: nil,
            origin: nil,
            heading_established: false

  def new(config) do
    imu_config = Keyword.fetch!(config, :imu_config)
    imu_type = Keyword.fetch!(imu_config, :imu_type)
    imu_parameters = Keyword.fetch!(imu_config, :imu_parameters)

    %ViaEstimation.Ekf.SevenState{
      ekf_state: {0, 0, 0, 0, 0, 0, 0},
      ekf_cov: generate_ekf_cov(config),
      r_gps: generate_r_gps(config),
      r_heading: generate_r_heading(config),
      q_ekf: generate_q(config),
      imu: apply(imu_type, :new, [imu_parameters])
    }
  end

  def predict(state, dt_accel_gyro) do
    %{
      SVN.accel_x_mpss() => ax_mpss,
      SVN.accel_y_mpss() => ay_mpss,
      SVN.accel_z_mpss() => az_mpss,
      SVN.dt_s() => dt_s
    } = dt_accel_gyro

    # IO.puts("q_ekf: #{inspect(state.q_ekf)}")
    imu = ViaEstimation.Imu.Mahony.update(state.imu, dt_accel_gyro)
    %{roll_rad: roll_rad, pitch_rad: pitch_rad, yaw_rad: yaw_rad} = imu
    # Calculcate rbg_prime
    cosphi = :math.cos(roll_rad)
    sinphi = :math.sin(roll_rad)
    costheta = :math.cos(pitch_rad)
    sintheta = :math.sin(pitch_rad)
    cospsi = :math.cos(yaw_rad)
    sinpsi = :math.sin(yaw_rad)

    rbgp00 = -costheta * sinpsi
    rbgp01 = -sinphi * sintheta * sinpsi - cosphi * cospsi
    rbgp02 = -cosphi * sintheta * sinpsi + sinphi * cospsi
    rbgp10 = costheta * cospsi
    rbgp11 = sinphi * sintheta * cospsi - cosphi * sinpsi
    rbgp12 = cosphi * sintheta * cospsi + sinphi * sinpsi
    rbgp20 = 0
    rbgp21 = 0
    rbgp22 = 0
    # Calculate inertial accel
    accel_inertial_x =
      az_mpss * (sinphi * sinpsi + cosphi * cospsi * sintheta) -
        ay_mpss * (cosphi * sinpsi - cospsi * sinphi * sintheta) + ax_mpss * cospsi * costheta

    accel_inertial_y =
      ay_mpss * (cosphi * cospsi + sinphi * sinpsi * sintheta) -
        az_mpss * (cospsi * sinphi - cosphi * sinpsi * sintheta) + ax_mpss * costheta * sinpsi

    accel_inertial_z =
      az_mpss * cosphi * costheta - ax_mpss * sintheta + ay_mpss * costheta * sinphi

    {ekfs0, ekfs1, ekfs2, ekfs3, ekfs4, ekfs5, _ekfs6} = state.ekf_state

    ekf_state =
      {ekfs0 + ekfs3 * dt_s, ekfs1 + ekfs4 * dt_s, ekfs2 + ekfs5 * dt_s,
       ekfs3 + accel_inertial_x * dt_s, ekfs4 + accel_inertial_y * dt_s,
       ekfs5 + (accel_inertial_z + VC.gravity()) * dt_s, yaw_rad}

    # gps = g_prime_submatrix
    gps0 = rbgp00 * ax_mpss + rbgp01 * ay_mpss + rbgp02 * az_mpss
    gps1 = rbgp10 * ax_mpss + rbgp11 * ay_mpss + rbgp12 * az_mpss
    gps2 = rbgp20 * ax_mpss + rbgp21 * ay_mpss + rbgp22 * az_mpss

    {ekfcov00, ekfcov01, ekfcov02, ekfcov03, ekfcov04, ekfcov05, ekfcov06, ekfcov10, ekfcov11,
     ekfcov12, ekfcov13, ekfcov14, ekfcov15, ekfcov16, ekfcov20, ekfcov21, ekfcov22, ekfcov23,
     ekfcov24, ekfcov25, ekfcov26, ekfcov30, ekfcov31, ekfcov32, ekfcov33, ekfcov34, ekfcov35,
     ekfcov36, ekfcov40, ekfcov41, ekfcov42, ekfcov43, ekfcov44, ekfcov45, ekfcov46, ekfcov50,
     ekfcov51, ekfcov52, ekfcov53, ekfcov54, ekfcov55, ekfcov56, ekfcov60, ekfcov61, ekfcov62,
     ekfcov63, ekfcov64, ekfcov65, ekfcov66} = state.ekf_cov

    c00 = ekfcov00 + dt_s * ekfcov30
    c01 = ekfcov01 + dt_s * ekfcov31
    c02 = ekfcov02 + dt_s * ekfcov32
    c03 = ekfcov03 + dt_s * ekfcov33
    c04 = ekfcov04 + dt_s * ekfcov34
    c05 = ekfcov05 + dt_s * ekfcov35
    c06 = ekfcov06 + dt_s * ekfcov36
    c10 = ekfcov10 + dt_s * ekfcov40
    c11 = ekfcov11 + dt_s * ekfcov41
    c12 = ekfcov12 + dt_s * ekfcov42
    c13 = ekfcov13 + dt_s * ekfcov43
    c14 = ekfcov14 + dt_s * ekfcov44
    c15 = ekfcov15 + dt_s * ekfcov45
    c16 = ekfcov16 + dt_s * ekfcov46
    c20 = ekfcov20 + dt_s * ekfcov50
    c21 = ekfcov21 + dt_s * ekfcov51
    c22 = ekfcov22 + dt_s * ekfcov52
    c23 = ekfcov23 + dt_s * ekfcov53
    c24 = ekfcov24 + dt_s * ekfcov54
    c25 = ekfcov25 + dt_s * ekfcov55
    c26 = ekfcov26 + dt_s * ekfcov56
    c30 = ekfcov30 + ekfcov60 * gps0
    c31 = ekfcov31 + ekfcov61 * gps0
    c32 = ekfcov32 + ekfcov62 * gps0
    c33 = ekfcov33 + ekfcov63 * gps0
    c34 = ekfcov34 + ekfcov64 * gps0
    c35 = ekfcov35 + ekfcov65 * gps0
    c36 = ekfcov36 + ekfcov66 * gps0
    c40 = ekfcov40 + ekfcov60 * gps1
    c41 = ekfcov41 + ekfcov61 * gps1
    c42 = ekfcov42 + ekfcov62 * gps1
    c43 = ekfcov43 + ekfcov63 * gps1
    c44 = ekfcov44 + ekfcov64 * gps1
    c45 = ekfcov45 + ekfcov65 * gps1
    c46 = ekfcov46 + ekfcov66 * gps1
    c50 = ekfcov50 + ekfcov60 * gps2
    c51 = ekfcov51 + ekfcov61 * gps2
    c52 = ekfcov52 + ekfcov62 * gps2
    c53 = ekfcov53 + ekfcov63 * gps2
    c54 = ekfcov54 + ekfcov64 * gps2
    c55 = ekfcov55 + ekfcov65 * gps2
    c56 = ekfcov56 + ekfcov66 * gps2
    c60 = ekfcov60
    c61 = ekfcov61
    c62 = ekfcov62
    c63 = ekfcov63
    c64 = ekfcov64
    c65 = ekfcov65
    c66 = ekfcov66

    {q00, q11, q22, q33, q44, q55, q66} = state.q_ekf

    ekf_cov =
      {c00 + q00 + c03 * dt_s, c01 + c04 * dt_s, c02 + c05 * dt_s, c03 + c06 * gps0,
       c04 + c06 * gps1, c05 + c06 * gps2, c06, c10 + c13 * dt_s, c11 + q11 + c14 * dt_s,
       c12 + c15 * dt_s, c13 + c16 * gps0, c14 + c16 * gps1, c15 + c16 * gps2, c16,
       c20 + c23 * dt_s, c21 + c24 * dt_s, c22 + q22 + c25 * dt_s, c23 + c26 * gps0,
       c24 + c26 * gps1, c25 + c26 * gps2, c26, c30 + c33 * dt_s, c31 + c34 * dt_s,
       c32 + c35 * dt_s, c33 + q33 + c36 * gps0, c34 + c36 * gps1, c35 + c36 * gps2, c36,
       c40 + c43 * dt_s, c41 + c44 * dt_s, c42 + c45 * dt_s, c43 + c46 * gps0,
       c44 + q44 + c46 * gps1, c45 + c46 * gps2, c46, c50 + c53 * dt_s, c51 + c54 * dt_s,
       c52 + c55 * dt_s, c53 + c56 * gps0, c54 + c56 * gps1, c55 + q55 + c56 * gps2, c56,
       c60 + c63 * dt_s, c61 + c64 * dt_s, c62 + c65 * dt_s, c63 + c66 * gps0, c64 + c66 * gps1,
       c65 + c66 * gps2, c66 + q66}

    %{state | imu: imu, ekf_state: ekf_state, ekf_cov: ekf_cov}
  end

  def update_from_gps(state, position_rrm, velocity_mps) do
    %{
      SVN.v_north_mps() => v_north_mps,
      SVN.v_east_mps() => v_east_mps,
      SVN.v_down_mps() => v_down_mps
    } = velocity_mps

    altitude_m = -Map.fetch(position_rrm, SVN.altitude_m())

    origin =
      if is_nil(state.origin) do
        position_rrm |> Map.put(SVN.altitude_m(), altitude_m)
      else
        state.origin
      end

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(origin, position_rrm)

    dz = altitude_m - origin.altitude_m

    {r00, r11, r22, r33, r44, r55} = state.r_gps

    {ekfcov00, ekfcov01, ekfcov02, ekfcov03, ekfcov04, ekfcov05, ekfcov06, ekfcov10, ekfcov11,
     ekfcov12, ekfcov13, ekfcov14, ekfcov15, ekfcov16, ekfcov20, ekfcov21, ekfcov22, ekfcov23,
     ekfcov24, ekfcov25, ekfcov26, ekfcov30, ekfcov31, ekfcov32, ekfcov33, ekfcov34, ekfcov35,
     ekfcov36, ekfcov40, ekfcov41, ekfcov42, ekfcov43, ekfcov44, ekfcov45, ekfcov46, ekfcov50,
     ekfcov51, ekfcov52, ekfcov53, ekfcov54, ekfcov55, ekfcov56, ekfcov60, ekfcov61, ekfcov62,
     ekfcov63, ekfcov64, ekfcov65, ekfcov66} = state.ekf_cov

    # Using 1-based indexing for this, because it used to be with Matrex
    m11 = ekfcov00 + r00
    m12 = ekfcov01
    m13 = ekfcov02
    m14 = ekfcov03
    m15 = ekfcov04
    m16 = ekfcov05
    m21 = ekfcov10
    m22 = ekfcov11 + r11
    m23 = ekfcov12
    m24 = ekfcov13
    m25 = ekfcov14
    m26 = ekfcov15
    m31 = ekfcov20
    m32 = ekfcov21
    m33 = ekfcov22 + r22
    m34 = ekfcov23
    m35 = ekfcov24
    m36 = ekfcov25
    m41 = ekfcov30
    m42 = ekfcov31
    m43 = ekfcov32
    m44 = ekfcov33 + r33
    m45 = ekfcov34
    m46 = ekfcov35
    m51 = ekfcov40
    m52 = ekfcov41
    m53 = ekfcov42
    m54 = ekfcov43
    m55 = ekfcov44 + r44
    m56 = ekfcov45
    m61 = ekfcov50
    m62 = ekfcov51
    m63 = ekfcov52
    m64 = ekfcov53
    m65 = ekfcov54
    m66 = ekfcov55 + r55

    a4545 = m55 * m66 - m56 * m65
    a3545 = m54 * m66 - m56 * m64
    a3445 = m54 * m65 - m55 * m64
    a2545 = m53 * m66 - m56 * m63
    a2445 = m53 * m65 - m55 * m63
    a2345 = m53 * m64 - m54 * m63
    a1545 = m52 * m66 - m56 * m62
    a1445 = m52 * m65 - m55 * m62
    a1345 = m52 * m64 - m54 * m62
    a1245 = m52 * m63 - m53 * m62
    a0545 = m51 * m66 - m56 * m61
    a0445 = m51 * m65 - m55 * m61
    a0345 = m51 * m64 - m54 * m61
    a0245 = m51 * m63 - m53 * m61
    a0145 = m51 * m62 - m52 * m61
    a4535 = m45 * m66 - m46 * m65
    a3535 = m44 * m66 - m46 * m64
    a3435 = m44 * m65 - m45 * m64
    a2535 = m43 * m66 - m46 * m63
    a2435 = m43 * m65 - m45 * m63
    a2335 = m43 * m64 - m44 * m63
    a1535 = m42 * m66 - m46 * m62
    a1435 = m42 * m65 - m45 * m62
    a1335 = m42 * m64 - m44 * m62
    a1235 = m42 * m63 - m43 * m62
    a4534 = m45 * m56 - m46 * m55
    a3534 = m44 * m56 - m46 * m54
    a3434 = m44 * m55 - m45 * m54
    a2534 = m43 * m56 - m46 * m53
    a2434 = m43 * m55 - m45 * m53
    a2334 = m43 * m54 - m44 * m53
    a1534 = m42 * m56 - m46 * m52
    a1434 = m42 * m55 - m45 * m52
    a1334 = m42 * m54 - m44 * m52
    a1234 = m42 * m53 - m43 * m52
    a0535 = m41 * m66 - m46 * m61
    a0435 = m41 * m65 - m45 * m61
    a0335 = m41 * m64 - m44 * m61
    a0235 = m41 * m63 - m43 * m61
    a0534 = m41 * m56 - m46 * m51
    a0434 = m41 * m55 - m45 * m51
    a0334 = m41 * m54 - m44 * m51
    a0234 = m41 * m53 - m43 * m51
    a0135 = m41 * m62 - m42 * m61
    a0134 = m41 * m52 - m42 * m51

    b345345 = m44 * a4545 - m45 * a3545 + m46 * a3445
    b245345 = m43 * a4545 - m45 * a2545 + m46 * a2445
    b235345 = m43 * a3545 - m44 * a2545 + m46 * a2345
    b234345 = m43 * a3445 - m44 * a2445 + m45 * a2345
    b145345 = m42 * a4545 - m45 * a1545 + m46 * a1445
    b135345 = m42 * a3545 - m44 * a1545 + m46 * a1345
    b134345 = m42 * a3445 - m44 * a1445 + m45 * a1345
    b125345 = m42 * a2545 - m43 * a1545 + m46 * a1245
    b124345 = m42 * a2445 - m43 * a1445 + m45 * a1245
    b123345 = m42 * a2345 - m43 * a1345 + m44 * a1245
    b045345 = m41 * a4545 - m45 * a0545 + m46 * a0445
    b035345 = m41 * a3545 - m44 * a0545 + m46 * a0345
    b034345 = m41 * a3445 - m44 * a0445 + m45 * a0345
    b025345 = m41 * a2545 - m43 * a0545 + m46 * a0245
    b024345 = m41 * a2445 - m43 * a0445 + m45 * a0245
    b023345 = m41 * a2345 - m43 * a0345 + m44 * a0245
    b015345 = m41 * a1545 - m42 * a0545 + m46 * a0145
    b014345 = m41 * a1445 - m42 * a0445 + m45 * a0145
    b013345 = m41 * a1345 - m42 * a0345 + m44 * a0145
    b012345 = m41 * a1245 - m42 * a0245 + m43 * a0145
    b345245 = m34 * a4545 - m35 * a3545 + m36 * a3445
    b245245 = m33 * a4545 - m35 * a2545 + m36 * a2445
    b235245 = m33 * a3545 - m34 * a2545 + m36 * a2345
    b234245 = m33 * a3445 - m34 * a2445 + m35 * a2345
    b145245 = m32 * a4545 - m35 * a1545 + m36 * a1445
    b135245 = m32 * a3545 - m34 * a1545 + m36 * a1345
    b134245 = m32 * a3445 - m34 * a1445 + m35 * a1345
    b125245 = m32 * a2545 - m33 * a1545 + m36 * a1245
    b124245 = m32 * a2445 - m33 * a1445 + m35 * a1245
    b123245 = m32 * a2345 - m33 * a1345 + m34 * a1245
    b345235 = m34 * a4535 - m35 * a3535 + m36 * a3435
    b245235 = m33 * a4535 - m35 * a2535 + m36 * a2435
    b235235 = m33 * a3535 - m34 * a2535 + m36 * a2335
    b234235 = m33 * a3435 - m34 * a2435 + m35 * a2335
    b145235 = m32 * a4535 - m35 * a1535 + m36 * a1435
    b135235 = m32 * a3535 - m34 * a1535 + m36 * a1335
    b134235 = m32 * a3435 - m34 * a1435 + m35 * a1335
    b125235 = m32 * a2535 - m33 * a1535 + m36 * a1235
    b124235 = m32 * a2435 - m33 * a1435 + m35 * a1235
    b123235 = m32 * a2335 - m33 * a1335 + m34 * a1235
    b345234 = m34 * a4534 - m35 * a3534 + m36 * a3434
    b245234 = m33 * a4534 - m35 * a2534 + m36 * a2434
    b235234 = m33 * a3534 - m34 * a2534 + m36 * a2334
    b234234 = m33 * a3434 - m34 * a2434 + m35 * a2334
    b145234 = m32 * a4534 - m35 * a1534 + m36 * a1434
    b135234 = m32 * a3534 - m34 * a1534 + m36 * a1334
    b134234 = m32 * a3434 - m34 * a1434 + m35 * a1334
    b125234 = m32 * a2534 - m33 * a1534 + m36 * a1234
    b124234 = m32 * a2434 - m33 * a1434 + m35 * a1234
    b123234 = m32 * a2334 - m33 * a1334 + m34 * a1234
    b045245 = m31 * a4545 - m35 * a0545 + m36 * a0445
    b035245 = m31 * a3545 - m34 * a0545 + m36 * a0345
    b034245 = m31 * a3445 - m34 * a0445 + m35 * a0345
    b025245 = m31 * a2545 - m33 * a0545 + m36 * a0245
    b024245 = m31 * a2445 - m33 * a0445 + m35 * a0245
    b023245 = m31 * a2345 - m33 * a0345 + m34 * a0245
    b045235 = m31 * a4535 - m35 * a0535 + m36 * a0435
    b035235 = m31 * a3535 - m34 * a0535 + m36 * a0335
    b034235 = m31 * a3435 - m34 * a0435 + m35 * a0335
    b025235 = m31 * a2535 - m33 * a0535 + m36 * a0235
    b024235 = m31 * a2435 - m33 * a0435 + m35 * a0235
    b023235 = m31 * a2335 - m33 * a0335 + m34 * a0235
    b045234 = m31 * a4534 - m35 * a0534 + m36 * a0434
    b035234 = m31 * a3534 - m34 * a0534 + m36 * a0334
    b034234 = m31 * a3434 - m34 * a0434 + m35 * a0334
    b025234 = m31 * a2534 - m33 * a0534 + m36 * a0234
    b024234 = m31 * a2434 - m33 * a0434 + m35 * a0234
    b023234 = m31 * a2334 - m33 * a0334 + m34 * a0234
    b015245 = m31 * a1545 - m32 * a0545 + m36 * a0145
    b014245 = m31 * a1445 - m32 * a0445 + m35 * a0145
    b013245 = m31 * a1345 - m32 * a0345 + m34 * a0145
    b015235 = m31 * a1535 - m32 * a0535 + m36 * a0135
    b014235 = m31 * a1435 - m32 * a0435 + m35 * a0135
    b013235 = m31 * a1335 - m32 * a0335 + m34 * a0135
    b015234 = m31 * a1534 - m32 * a0534 + m36 * a0134
    b014234 = m31 * a1434 - m32 * a0434 + m35 * a0134
    b013234 = m31 * a1334 - m32 * a0334 + m34 * a0134
    b012245 = m31 * a1245 - m32 * a0245 + m33 * a0145
    b012235 = m31 * a1235 - m32 * a0235 + m33 * a0135
    b012234 = m31 * a1234 - m32 * a0234 + m33 * a0134

    c23452345 = m33 * b345345 - m34 * b245345 + m35 * b235345 - m36 * b234345
    c13452345 = m32 * b345345 - m34 * b145345 + m35 * b135345 - m36 * b134345
    c12452345 = m32 * b245345 - m33 * b145345 + m35 * b125345 - m36 * b124345
    c12352345 = m32 * b235345 - m33 * b135345 + m34 * b125345 - m36 * b123345
    c12342345 = m32 * b234345 - m33 * b134345 + m34 * b124345 - m35 * b123345
    c03452345 = m31 * b345345 - m34 * b045345 + m35 * b035345 - m36 * b034345
    c02452345 = m31 * b245345 - m33 * b045345 + m35 * b025345 - m36 * b024345
    c02352345 = m31 * b235345 - m33 * b035345 + m34 * b025345 - m36 * b023345
    c02342345 = m31 * b234345 - m33 * b034345 + m34 * b024345 - m35 * b023345
    c01452345 = m31 * b145345 - m32 * b045345 + m35 * b015345 - m36 * b014345
    c01352345 = m31 * b135345 - m32 * b035345 + m34 * b015345 - m36 * b013345
    c01342345 = m31 * b134345 - m32 * b034345 + m34 * b014345 - m35 * b013345
    c01252345 = m31 * b125345 - m32 * b025345 + m33 * b015345 - m36 * b012345
    c01242345 = m31 * b124345 - m32 * b024345 + m33 * b014345 - m35 * b012345
    c01232345 = m31 * b123345 - m32 * b023345 + m33 * b013345 - m34 * b012345
    c23451345 = m23 * b345345 - m24 * b245345 + m25 * b235345 - m26 * b234345
    c13451345 = m22 * b345345 - m24 * b145345 + m25 * b135345 - m26 * b134345
    c12451345 = m22 * b245345 - m23 * b145345 + m25 * b125345 - m26 * b124345
    c12351345 = m22 * b235345 - m23 * b135345 + m24 * b125345 - m26 * b123345
    c12341345 = m22 * b234345 - m23 * b134345 + m24 * b124345 - m25 * b123345
    c23451245 = m23 * b345245 - m24 * b245245 + m25 * b235245 - m26 * b234245
    c13451245 = m22 * b345245 - m24 * b145245 + m25 * b135245 - m26 * b134245
    c12451245 = m22 * b245245 - m23 * b145245 + m25 * b125245 - m26 * b124245
    c12351245 = m22 * b235245 - m23 * b135245 + m24 * b125245 - m26 * b123245
    c12341245 = m22 * b234245 - m23 * b134245 + m24 * b124245 - m25 * b123245
    c23451235 = m23 * b345235 - m24 * b245235 + m25 * b235235 - m26 * b234235
    c13451235 = m22 * b345235 - m24 * b145235 + m25 * b135235 - m26 * b134235
    c12451235 = m22 * b245235 - m23 * b145235 + m25 * b125235 - m26 * b124235
    c12351235 = m22 * b235235 - m23 * b135235 + m24 * b125235 - m26 * b123235
    c12341235 = m22 * b234235 - m23 * b134235 + m24 * b124235 - m25 * b123235
    c23451234 = m23 * b345234 - m24 * b245234 + m25 * b235234 - m26 * b234234
    c13451234 = m22 * b345234 - m24 * b145234 + m25 * b135234 - m26 * b134234
    c12451234 = m22 * b245234 - m23 * b145234 + m25 * b125234 - m26 * b124234
    c12351234 = m22 * b235234 - m23 * b135234 + m24 * b125234 - m26 * b123234
    c12341234 = m22 * b234234 - m23 * b134234 + m24 * b124234 - m25 * b123234
    c03451345 = m21 * b345345 - m24 * b045345 + m25 * b035345 - m26 * b034345
    c02451345 = m21 * b245345 - m23 * b045345 + m25 * b025345 - m26 * b024345
    c02351345 = m21 * b235345 - m23 * b035345 + m24 * b025345 - m26 * b023345
    c02341345 = m21 * b234345 - m23 * b034345 + m24 * b024345 - m25 * b023345
    c03451245 = m21 * b345245 - m24 * b045245 + m25 * b035245 - m26 * b034245
    c02451245 = m21 * b245245 - m23 * b045245 + m25 * b025245 - m26 * b024245
    c02351245 = m21 * b235245 - m23 * b035245 + m24 * b025245 - m26 * b023245
    c02341245 = m21 * b234245 - m23 * b034245 + m24 * b024245 - m25 * b023245
    c03451235 = m21 * b345235 - m24 * b045235 + m25 * b035235 - m26 * b034235
    c02451235 = m21 * b245235 - m23 * b045235 + m25 * b025235 - m26 * b024235
    c02351235 = m21 * b235235 - m23 * b035235 + m24 * b025235 - m26 * b023235
    c02341235 = m21 * b234235 - m23 * b034235 + m24 * b024235 - m25 * b023235
    c03451234 = m21 * b345234 - m24 * b045234 + m25 * b035234 - m26 * b034234
    c02451234 = m21 * b245234 - m23 * b045234 + m25 * b025234 - m26 * b024234
    c02351234 = m21 * b235234 - m23 * b035234 + m24 * b025234 - m26 * b023234
    c02341234 = m21 * b234234 - m23 * b034234 + m24 * b024234 - m25 * b023234
    c01451345 = m21 * b145345 - m22 * b045345 + m25 * b015345 - m26 * b014345
    c01351345 = m21 * b135345 - m22 * b035345 + m24 * b015345 - m26 * b013345
    c01341345 = m21 * b134345 - m22 * b034345 + m24 * b014345 - m25 * b013345
    c01451245 = m21 * b145245 - m22 * b045245 + m25 * b015245 - m26 * b014245
    c01351245 = m21 * b135245 - m22 * b035245 + m24 * b015245 - m26 * b013245
    c01341245 = m21 * b134245 - m22 * b034245 + m24 * b014245 - m25 * b013245
    c01451235 = m21 * b145235 - m22 * b045235 + m25 * b015235 - m26 * b014235
    c01351235 = m21 * b135235 - m22 * b035235 + m24 * b015235 - m26 * b013235
    c01341235 = m21 * b134235 - m22 * b034235 + m24 * b014235 - m25 * b013235
    c01451234 = m21 * b145234 - m22 * b045234 + m25 * b015234 - m26 * b014234
    c01351234 = m21 * b135234 - m22 * b035234 + m24 * b015234 - m26 * b013234
    c01341234 = m21 * b134234 - m22 * b034234 + m24 * b014234 - m25 * b013234
    c01251345 = m21 * b125345 - m22 * b025345 + m23 * b015345 - m26 * b012345
    c01241345 = m21 * b124345 - m22 * b024345 + m23 * b014345 - m25 * b012345
    c01251245 = m21 * b125245 - m22 * b025245 + m23 * b015245 - m26 * b012245
    c01241245 = m21 * b124245 - m22 * b024245 + m23 * b014245 - m25 * b012245
    c01251235 = m21 * b125235 - m22 * b025235 + m23 * b015235 - m26 * b012235
    c01241235 = m21 * b124235 - m22 * b024235 + m23 * b014235 - m25 * b012235
    c01251234 = m21 * b125234 - m22 * b025234 + m23 * b015234 - m26 * b012234
    c01241234 = m21 * b124234 - m22 * b024234 + m23 * b014234 - m25 * b012234
    c01231345 = m21 * b123345 - m22 * b023345 + m23 * b013345 - m24 * b012345
    c01231245 = m21 * b123245 - m22 * b023245 + m23 * b013245 - m24 * b012245
    c01231235 = m21 * b123235 - m22 * b023235 + m23 * b013235 - m24 * b012235
    c01231234 = m21 * b123234 - m22 * b023234 + m23 * b013234 - m24 * b012234

    det =
      (m11 *
         (m22 * c23452345 - m23 * c13452345 + m24 * c12452345 - m25 * c12352345 +
            m26 * c12342345))
      |> Kernel.+(
        -m12 *
          (m21 * c23452345 - m23 * c03452345 + m24 * c02452345 - m25 * c02352345 +
             m26 * c02342345)
      )
      |> Kernel.+(
        m13 *
          (m21 * c13452345 - m22 * c03452345 + m24 * c01452345 - m25 * c01352345 +
             m26 * c01342345)
      )
      |> Kernel.+(
        -m14 *
          (m21 * c12452345 - m22 * c02452345 + m23 * c01452345 - m25 * c01252345 +
             m26 * c01242345)
      )
      |> Kernel.+(
        m15 *
          (m21 * c12352345 - m22 * c02352345 + m23 * c01352345 - m24 * c01252345 +
             m26 * c01232345)
      )
      |> Kernel.+(
        -m16 *
          (m21 * c12342345 - m22 * c02342345 + m23 * c01342345 - m24 * c01242345 +
             m25 * c01232345)
      )

    det = if det != 0, do: 1 / det, else: 0

    inv_mat00 =
      det *
        (m22 * c23452345 - m23 * c13452345 + m24 * c12452345 - m25 * c12352345 +
           m26 * c12342345)

    inv_mat01 =
      det *
        -(m12 * c23452345 - m13 * c13452345 + m14 * c12452345 - m15 * c12352345 +
            m16 * c12342345)

    inv_mat02 =
      det *
        (m12 * c23451345 - m13 * c13451345 + m14 * c12451345 - m15 * c12351345 +
           m16 * c12341345)

    inv_mat03 =
      det *
        -(m12 * c23451245 - m13 * c13451245 + m14 * c12451245 - m15 * c12351245 +
            m16 * c12341245)

    inv_mat04 =
      det *
        (m12 * c23451235 - m13 * c13451235 + m14 * c12451235 - m15 * c12351235 +
           m16 * c12341235)

    inv_mat05 =
      det *
        -(m12 * c23451234 - m13 * c13451234 + m14 * c12451234 - m15 * c12351234 +
            m16 * c12341234)

    inv_mat10 =
      det *
        -(m21 * c23452345 - m23 * c03452345 + m24 * c02452345 - m25 * c02352345 +
            m26 * c02342345)

    inv_mat11 =
      det *
        (m11 * c23452345 - m13 * c03452345 + m14 * c02452345 - m15 * c02352345 +
           m16 * c02342345)

    inv_mat12 =
      det *
        -(m11 * c23451345 - m13 * c03451345 + m14 * c02451345 - m15 * c02351345 +
            m16 * c02341345)

    inv_mat13 =
      det *
        (m11 * c23451245 - m13 * c03451245 + m14 * c02451245 - m15 * c02351245 +
           m16 * c02341245)

    inv_mat14 =
      det *
        -(m11 * c23451235 - m13 * c03451235 + m14 * c02451235 - m15 * c02351235 +
            m16 * c02341235)

    inv_mat15 =
      det *
        (m11 * c23451234 - m13 * c03451234 + m14 * c02451234 - m15 * c02351234 +
           m16 * c02341234)

    inv_mat20 =
      det *
        (m21 * c13452345 - m22 * c03452345 + m24 * c01452345 - m25 * c01352345 +
           m26 * c01342345)

    inv_mat21 =
      det *
        -(m11 * c13452345 - m12 * c03452345 + m14 * c01452345 - m15 * c01352345 +
            m16 * c01342345)

    inv_mat22 =
      det *
        (m11 * c13451345 - m12 * c03451345 + m14 * c01451345 - m15 * c01351345 +
           m16 * c01341345)

    inv_mat23 =
      det *
        -(m11 * c13451245 - m12 * c03451245 + m14 * c01451245 - m15 * c01351245 +
            m16 * c01341245)

    inv_mat24 =
      det *
        (m11 * c13451235 - m12 * c03451235 + m14 * c01451235 - m15 * c01351235 +
           m16 * c01341235)

    inv_mat25 =
      det *
        -(m11 * c13451234 - m12 * c03451234 + m14 * c01451234 - m15 * c01351234 +
            m16 * c01341234)

    inv_mat30 =
      det *
        -(m21 * c12452345 - m22 * c02452345 + m23 * c01452345 - m25 * c01252345 +
            m26 * c01242345)

    inv_mat31 =
      det *
        (m11 * c12452345 - m12 * c02452345 + m13 * c01452345 - m15 * c01252345 +
           m16 * c01242345)

    inv_mat32 =
      det *
        -(m11 * c12451345 - m12 * c02451345 + m13 * c01451345 - m15 * c01251345 +
            m16 * c01241345)

    inv_mat33 =
      det *
        (m11 * c12451245 - m12 * c02451245 + m13 * c01451245 - m15 * c01251245 +
           m16 * c01241245)

    inv_mat34 =
      det *
        -(m11 * c12451235 - m12 * c02451235 + m13 * c01451235 - m15 * c01251235 +
            m16 * c01241235)

    inv_mat35 =
      det *
        (m11 * c12451234 - m12 * c02451234 + m13 * c01451234 - m15 * c01251234 +
           m16 * c01241234)

    inv_mat40 =
      det *
        (m21 * c12352345 - m22 * c02352345 + m23 * c01352345 - m24 * c01252345 +
           m26 * c01232345)

    inv_mat41 =
      det *
        -(m11 * c12352345 - m12 * c02352345 + m13 * c01352345 - m14 * c01252345 +
            m16 * c01232345)

    inv_mat42 =
      det *
        (m11 * c12351345 - m12 * c02351345 + m13 * c01351345 - m14 * c01251345 +
           m16 * c01231345)

    inv_mat43 =
      det *
        -(m11 * c12351245 - m12 * c02351245 + m13 * c01351245 - m14 * c01251245 +
            m16 * c01231245)

    inv_mat44 =
      det *
        (m11 * c12351235 - m12 * c02351235 + m13 * c01351235 - m14 * c01251235 +
           m16 * c01231235)

    inv_mat45 =
      det *
        -(m11 * c12351234 - m12 * c02351234 + m13 * c01351234 - m14 * c01251234 +
            m16 * c01231234)

    inv_mat50 =
      det *
        -(m21 * c12342345 - m22 * c02342345 + m23 * c01342345 - m24 * c01242345 +
            m25 * c01232345)

    inv_mat51 =
      det *
        (m11 * c12342345 - m12 * c02342345 + m13 * c01342345 - m14 * c01242345 +
           m15 * c01232345)

    inv_mat52 =
      det *
        -(m11 * c12341345 - m12 * c02341345 + m13 * c01341345 - m14 * c01241345 +
            m15 * c01231345)

    inv_mat53 =
      det *
        (m11 * c12341245 - m12 * c02341245 + m13 * c01341245 - m14 * c01241245 +
           m15 * c01231245)

    inv_mat54 =
      det *
        -(m11 * c12341235 - m12 * c02341235 + m13 * c01341235 - m14 * c01241235 +
            m15 * c01231235)

    inv_mat55 =
      det *
        (m11 * c12341234 - m12 * c02341234 + m13 * c01341234 - m14 * c01241234 +
           m15 * c01231234)

    k00 =
      ekfcov00 * inv_mat00 + ekfcov01 * inv_mat10 + ekfcov02 * inv_mat20 +
        ekfcov03 * inv_mat30 + ekfcov04 * inv_mat40 + ekfcov05 * inv_mat50

    k01 =
      ekfcov00 * inv_mat01 + ekfcov01 * inv_mat11 + ekfcov02 * inv_mat21 +
        ekfcov03 * inv_mat31 + ekfcov04 * inv_mat41 + ekfcov05 * inv_mat51

    k02 =
      ekfcov00 * inv_mat02 + ekfcov01 * inv_mat12 + ekfcov02 * inv_mat22 +
        ekfcov03 * inv_mat32 + ekfcov04 * inv_mat42 + ekfcov05 * inv_mat52

    k03 =
      ekfcov00 * inv_mat03 + ekfcov01 * inv_mat13 + ekfcov02 * inv_mat23 +
        ekfcov03 * inv_mat33 + ekfcov04 * inv_mat43 + ekfcov05 * inv_mat53

    k04 =
      ekfcov00 * inv_mat04 + ekfcov01 * inv_mat14 + ekfcov02 * inv_mat24 +
        ekfcov03 * inv_mat34 + ekfcov04 * inv_mat44 + ekfcov05 * inv_mat54

    k05 =
      ekfcov00 * inv_mat05 + ekfcov01 * inv_mat15 + ekfcov02 * inv_mat25 +
        ekfcov03 * inv_mat35 + ekfcov04 * inv_mat45 + ekfcov05 * inv_mat55

    k10 =
      ekfcov10 * inv_mat00 + ekfcov11 * inv_mat10 + ekfcov12 * inv_mat20 +
        ekfcov13 * inv_mat30 + ekfcov14 * inv_mat40 + ekfcov15 * inv_mat50

    k11 =
      ekfcov10 * inv_mat01 + ekfcov11 * inv_mat11 + ekfcov12 * inv_mat21 +
        ekfcov13 * inv_mat31 + ekfcov14 * inv_mat41 + ekfcov15 * inv_mat51

    k12 =
      ekfcov10 * inv_mat02 + ekfcov11 * inv_mat12 + ekfcov12 * inv_mat22 +
        ekfcov13 * inv_mat32 + ekfcov14 * inv_mat42 + ekfcov15 * inv_mat52

    k13 =
      ekfcov10 * inv_mat03 + ekfcov11 * inv_mat13 + ekfcov12 * inv_mat23 +
        ekfcov13 * inv_mat33 + ekfcov14 * inv_mat43 + ekfcov15 * inv_mat53

    k14 =
      ekfcov10 * inv_mat04 + ekfcov11 * inv_mat14 + ekfcov12 * inv_mat24 +
        ekfcov13 * inv_mat34 + ekfcov14 * inv_mat44 + ekfcov15 * inv_mat54

    k15 =
      ekfcov10 * inv_mat05 + ekfcov11 * inv_mat15 + ekfcov12 * inv_mat25 +
        ekfcov13 * inv_mat35 + ekfcov14 * inv_mat45 + ekfcov15 * inv_mat55

    k20 =
      ekfcov20 * inv_mat00 + ekfcov21 * inv_mat10 + ekfcov22 * inv_mat20 +
        ekfcov23 * inv_mat30 + ekfcov24 * inv_mat40 + ekfcov25 * inv_mat50

    k21 =
      ekfcov20 * inv_mat01 + ekfcov21 * inv_mat11 + ekfcov22 * inv_mat21 +
        ekfcov23 * inv_mat31 + ekfcov24 * inv_mat41 + ekfcov25 * inv_mat51

    k22 =
      ekfcov20 * inv_mat02 + ekfcov21 * inv_mat12 + ekfcov22 * inv_mat22 +
        ekfcov23 * inv_mat32 + ekfcov24 * inv_mat42 + ekfcov25 * inv_mat52

    k23 =
      ekfcov20 * inv_mat03 + ekfcov21 * inv_mat13 + ekfcov22 * inv_mat23 +
        ekfcov23 * inv_mat33 + ekfcov24 * inv_mat43 + ekfcov25 * inv_mat53

    k24 =
      ekfcov20 * inv_mat04 + ekfcov21 * inv_mat14 + ekfcov22 * inv_mat24 +
        ekfcov23 * inv_mat34 + ekfcov24 * inv_mat44 + ekfcov25 * inv_mat54

    k25 =
      ekfcov20 * inv_mat05 + ekfcov21 * inv_mat15 + ekfcov22 * inv_mat25 +
        ekfcov23 * inv_mat35 + ekfcov24 * inv_mat45 + ekfcov25 * inv_mat55

    k30 =
      ekfcov30 * inv_mat00 + ekfcov31 * inv_mat10 + ekfcov32 * inv_mat20 +
        ekfcov33 * inv_mat30 + ekfcov34 * inv_mat40 + ekfcov35 * inv_mat50

    k31 =
      ekfcov30 * inv_mat01 + ekfcov31 * inv_mat11 + ekfcov32 * inv_mat21 +
        ekfcov33 * inv_mat31 + ekfcov34 * inv_mat41 + ekfcov35 * inv_mat51

    k32 =
      ekfcov30 * inv_mat02 + ekfcov31 * inv_mat12 + ekfcov32 * inv_mat22 +
        ekfcov33 * inv_mat32 + ekfcov34 * inv_mat42 + ekfcov35 * inv_mat52

    k33 =
      ekfcov30 * inv_mat03 + ekfcov31 * inv_mat13 + ekfcov32 * inv_mat23 +
        ekfcov33 * inv_mat33 + ekfcov34 * inv_mat43 + ekfcov35 * inv_mat53

    k34 =
      ekfcov30 * inv_mat04 + ekfcov31 * inv_mat14 + ekfcov32 * inv_mat24 +
        ekfcov33 * inv_mat34 + ekfcov34 * inv_mat44 + ekfcov35 * inv_mat54

    k35 =
      ekfcov30 * inv_mat05 + ekfcov31 * inv_mat15 + ekfcov32 * inv_mat25 +
        ekfcov33 * inv_mat35 + ekfcov34 * inv_mat45 + ekfcov35 * inv_mat55

    k40 =
      ekfcov40 * inv_mat00 + ekfcov41 * inv_mat10 + ekfcov42 * inv_mat20 +
        ekfcov43 * inv_mat30 + ekfcov44 * inv_mat40 + ekfcov45 * inv_mat50

    k41 =
      ekfcov40 * inv_mat01 + ekfcov41 * inv_mat11 + ekfcov42 * inv_mat21 +
        ekfcov43 * inv_mat31 + ekfcov44 * inv_mat41 + ekfcov45 * inv_mat51

    k42 =
      ekfcov40 * inv_mat02 + ekfcov41 * inv_mat12 + ekfcov42 * inv_mat22 +
        ekfcov43 * inv_mat32 + ekfcov44 * inv_mat42 + ekfcov45 * inv_mat52

    k43 =
      ekfcov40 * inv_mat03 + ekfcov41 * inv_mat13 + ekfcov42 * inv_mat23 +
        ekfcov43 * inv_mat33 + ekfcov44 * inv_mat43 + ekfcov45 * inv_mat53

    k44 =
      ekfcov40 * inv_mat04 + ekfcov41 * inv_mat14 + ekfcov42 * inv_mat24 +
        ekfcov43 * inv_mat34 + ekfcov44 * inv_mat44 + ekfcov45 * inv_mat54

    k45 =
      ekfcov40 * inv_mat05 + ekfcov41 * inv_mat15 + ekfcov42 * inv_mat25 +
        ekfcov43 * inv_mat35 + ekfcov44 * inv_mat45 + ekfcov45 * inv_mat55

    k50 =
      ekfcov50 * inv_mat00 + ekfcov51 * inv_mat10 + ekfcov52 * inv_mat20 +
        ekfcov53 * inv_mat30 + ekfcov54 * inv_mat40 + ekfcov55 * inv_mat50

    k51 =
      ekfcov50 * inv_mat01 + ekfcov51 * inv_mat11 + ekfcov52 * inv_mat21 +
        ekfcov53 * inv_mat31 + ekfcov54 * inv_mat41 + ekfcov55 * inv_mat51

    k52 =
      ekfcov50 * inv_mat02 + ekfcov51 * inv_mat12 + ekfcov52 * inv_mat22 +
        ekfcov53 * inv_mat32 + ekfcov54 * inv_mat42 + ekfcov55 * inv_mat52

    k53 =
      ekfcov50 * inv_mat03 + ekfcov51 * inv_mat13 + ekfcov52 * inv_mat23 +
        ekfcov53 * inv_mat33 + ekfcov54 * inv_mat43 + ekfcov55 * inv_mat53

    k54 =
      ekfcov50 * inv_mat04 + ekfcov51 * inv_mat14 + ekfcov52 * inv_mat24 +
        ekfcov53 * inv_mat34 + ekfcov54 * inv_mat44 + ekfcov55 * inv_mat54

    k55 =
      ekfcov50 * inv_mat05 + ekfcov51 * inv_mat15 + ekfcov52 * inv_mat25 +
        ekfcov53 * inv_mat35 + ekfcov54 * inv_mat45 + ekfcov55 * inv_mat55

    k60 =
      ekfcov60 * inv_mat00 + ekfcov61 * inv_mat10 + ekfcov62 * inv_mat20 +
        ekfcov63 * inv_mat30 + ekfcov64 * inv_mat40 + ekfcov65 * inv_mat50

    k61 =
      ekfcov60 * inv_mat01 + ekfcov61 * inv_mat11 + ekfcov62 * inv_mat21 +
        ekfcov63 * inv_mat31 + ekfcov64 * inv_mat41 + ekfcov65 * inv_mat51

    k62 =
      ekfcov60 * inv_mat02 + ekfcov61 * inv_mat12 + ekfcov62 * inv_mat22 +
        ekfcov63 * inv_mat32 + ekfcov64 * inv_mat42 + ekfcov65 * inv_mat52

    k63 =
      ekfcov60 * inv_mat03 + ekfcov61 * inv_mat13 + ekfcov62 * inv_mat23 +
        ekfcov63 * inv_mat33 + ekfcov64 * inv_mat43 + ekfcov65 * inv_mat53

    k64 =
      ekfcov60 * inv_mat04 + ekfcov61 * inv_mat14 + ekfcov62 * inv_mat24 +
        ekfcov63 * inv_mat34 + ekfcov64 * inv_mat44 + ekfcov65 * inv_mat54

    k65 =
      ekfcov60 * inv_mat05 + ekfcov61 * inv_mat15 + ekfcov62 * inv_mat25 +
        ekfcov63 * inv_mat35 + ekfcov64 * inv_mat45 + ekfcov65 * inv_mat55

    {ekf_state0, ekf_state1, ekf_state2, ekf_state3, ekf_state4, ekf_state5, ekf_state6} =
      state.ekf_state

    dz0 = dx - ekf_state0
    dz1 = dy - ekf_state1
    dz2 = dz - ekf_state2
    dz3 = v_north_mps - ekf_state3
    dz4 = v_east_mps - ekf_state4
    dz5 = v_down_mps - ekf_state5

    ekf_state = {
      ekf_state0 + dz0 * k00 + dz1 * k01 + dz2 * k02 + dz3 * k03 + dz4 * k04 + dz5 * k05,
      ekf_state1 + dz0 * k10 + dz1 * k11 + dz2 * k12 + dz3 * k13 + dz4 * k14 + dz5 * k15,
      ekf_state2 + dz0 * k20 + dz1 * k21 + dz2 * k22 + dz3 * k23 + dz4 * k24 + dz5 * k25,
      ekf_state3 + dz0 * k30 + dz1 * k31 + dz2 * k32 + dz3 * k33 + dz4 * k34 + dz5 * k35,
      ekf_state4 + dz0 * k40 + dz1 * k41 + dz2 * k42 + dz3 * k43 + dz4 * k44 + dz5 * k45,
      ekf_state5 + dz0 * k50 + dz1 * k51 + dz2 * k52 + dz3 * k53 + dz4 * k54 + dz5 * k55,
      ekf_state6 + dz0 * k60 + dz1 * k61 + dz2 * k62 + dz3 * k63 + dz4 * k64 + dz5 * k65
    }

    ekf_cov =
      {-ekfcov10 * k01 - ekfcov20 * k02 - ekfcov30 * k03 - ekfcov40 * k04 - ekfcov50 * k05 -
         ekfcov00 * (k00 - 1),
       -ekfcov11 * k01 - ekfcov21 * k02 - ekfcov31 * k03 - ekfcov41 * k04 - ekfcov51 * k05 -
         ekfcov01 * (k00 - 1),
       -ekfcov12 * k01 - ekfcov22 * k02 - ekfcov32 * k03 - ekfcov42 * k04 - ekfcov52 * k05 -
         ekfcov02 * (k00 - 1),
       -ekfcov13 * k01 - ekfcov23 * k02 - ekfcov33 * k03 - ekfcov43 * k04 - ekfcov53 * k05 -
         ekfcov03 * (k00 - 1),
       -ekfcov14 * k01 - ekfcov24 * k02 - ekfcov34 * k03 - ekfcov44 * k04 - ekfcov54 * k05 -
         ekfcov04 * (k00 - 1),
       -ekfcov15 * k01 - ekfcov25 * k02 - ekfcov35 * k03 - ekfcov45 * k04 - ekfcov55 * k05 -
         ekfcov05 * (k00 - 1),
       -ekfcov16 * k01 - ekfcov26 * k02 - ekfcov36 * k03 - ekfcov46 * k04 - ekfcov56 * k05 -
         ekfcov06 * (k00 - 1),
       -ekfcov00 * k10 - ekfcov20 * k12 - ekfcov30 * k13 - ekfcov40 * k14 - ekfcov50 * k15 -
         ekfcov10 * (k11 - 1),
       -ekfcov01 * k10 - ekfcov21 * k12 - ekfcov31 * k13 - ekfcov41 * k14 - ekfcov51 * k15 -
         ekfcov11 * (k11 - 1),
       -ekfcov02 * k10 - ekfcov22 * k12 - ekfcov32 * k13 - ekfcov42 * k14 - ekfcov52 * k15 -
         ekfcov12 * (k11 - 1),
       -ekfcov03 * k10 - ekfcov23 * k12 - ekfcov33 * k13 - ekfcov43 * k14 - ekfcov53 * k15 -
         ekfcov13 * (k11 - 1),
       -ekfcov04 * k10 - ekfcov24 * k12 - ekfcov34 * k13 - ekfcov44 * k14 - ekfcov54 * k15 -
         ekfcov14 * (k11 - 1),
       -ekfcov05 * k10 - ekfcov25 * k12 - ekfcov35 * k13 - ekfcov45 * k14 - ekfcov55 * k15 -
         ekfcov15 * (k11 - 1),
       -ekfcov06 * k10 - ekfcov26 * k12 - ekfcov36 * k13 - ekfcov46 * k14 - ekfcov56 * k15 -
         ekfcov16 * (k11 - 1),
       -ekfcov00 * k20 - ekfcov10 * k21 - ekfcov30 * k23 - ekfcov40 * k24 - ekfcov50 * k25 -
         ekfcov20 * (k22 - 1),
       -ekfcov01 * k20 - ekfcov11 * k21 - ekfcov31 * k23 - ekfcov41 * k24 - ekfcov51 * k25 -
         ekfcov21 * (k22 - 1),
       -ekfcov02 * k20 - ekfcov12 * k21 - ekfcov32 * k23 - ekfcov42 * k24 - ekfcov52 * k25 -
         ekfcov22 * (k22 - 1),
       -ekfcov03 * k20 - ekfcov13 * k21 - ekfcov33 * k23 - ekfcov43 * k24 - ekfcov53 * k25 -
         ekfcov23 * (k22 - 1),
       -ekfcov04 * k20 - ekfcov14 * k21 - ekfcov34 * k23 - ekfcov44 * k24 - ekfcov54 * k25 -
         ekfcov24 * (k22 - 1),
       -ekfcov05 * k20 - ekfcov15 * k21 - ekfcov35 * k23 - ekfcov45 * k24 - ekfcov55 * k25 -
         ekfcov25 * (k22 - 1),
       -ekfcov06 * k20 - ekfcov16 * k21 - ekfcov36 * k23 - ekfcov46 * k24 - ekfcov56 * k25 -
         ekfcov26 * (k22 - 1),
       -ekfcov00 * k30 - ekfcov10 * k31 - ekfcov20 * k32 - ekfcov40 * k34 - ekfcov50 * k35 -
         ekfcov30 * (k33 - 1),
       -ekfcov01 * k30 - ekfcov11 * k31 - ekfcov21 * k32 - ekfcov41 * k34 - ekfcov51 * k35 -
         ekfcov31 * (k33 - 1),
       -ekfcov02 * k30 - ekfcov12 * k31 - ekfcov22 * k32 - ekfcov42 * k34 - ekfcov52 * k35 -
         ekfcov32 * (k33 - 1),
       -ekfcov03 * k30 - ekfcov13 * k31 - ekfcov23 * k32 - ekfcov43 * k34 - ekfcov53 * k35 -
         ekfcov33 * (k33 - 1),
       -ekfcov04 * k30 - ekfcov14 * k31 - ekfcov24 * k32 - ekfcov44 * k34 - ekfcov54 * k35 -
         ekfcov34 * (k33 - 1),
       -ekfcov05 * k30 - ekfcov15 * k31 - ekfcov25 * k32 - ekfcov45 * k34 - ekfcov55 * k35 -
         ekfcov35 * (k33 - 1),
       -ekfcov06 * k30 - ekfcov16 * k31 - ekfcov26 * k32 - ekfcov46 * k34 - ekfcov56 * k35 -
         ekfcov36 * (k33 - 1),
       -ekfcov00 * k40 - ekfcov10 * k41 - ekfcov20 * k42 - ekfcov30 * k43 - ekfcov50 * k45 -
         ekfcov40 * (k44 - 1),
       -ekfcov01 * k40 - ekfcov11 * k41 - ekfcov21 * k42 - ekfcov31 * k43 - ekfcov51 * k45 -
         ekfcov41 * (k44 - 1),
       -ekfcov02 * k40 - ekfcov12 * k41 - ekfcov22 * k42 - ekfcov32 * k43 - ekfcov52 * k45 -
         ekfcov42 * (k44 - 1),
       -ekfcov03 * k40 - ekfcov13 * k41 - ekfcov23 * k42 - ekfcov33 * k43 - ekfcov53 * k45 -
         ekfcov43 * (k44 - 1),
       -ekfcov04 * k40 - ekfcov14 * k41 - ekfcov24 * k42 - ekfcov34 * k43 - ekfcov54 * k45 -
         ekfcov44 * (k44 - 1),
       -ekfcov05 * k40 - ekfcov15 * k41 - ekfcov25 * k42 - ekfcov35 * k43 - ekfcov55 * k45 -
         ekfcov45 * (k44 - 1),
       -ekfcov06 * k40 - ekfcov16 * k41 - ekfcov26 * k42 - ekfcov36 * k43 - ekfcov56 * k45 -
         ekfcov46 * (k44 - 1),
       -ekfcov00 * k50 - ekfcov10 * k51 - ekfcov20 * k52 - ekfcov30 * k53 - ekfcov40 * k54 -
         ekfcov50 * (k55 - 1),
       -ekfcov01 * k50 - ekfcov11 * k51 - ekfcov21 * k52 - ekfcov31 * k53 - ekfcov41 * k54 -
         ekfcov51 * (k55 - 1),
       -ekfcov02 * k50 - ekfcov12 * k51 - ekfcov22 * k52 - ekfcov32 * k53 - ekfcov42 * k54 -
         ekfcov52 * (k55 - 1),
       -ekfcov03 * k50 - ekfcov13 * k51 - ekfcov23 * k52 - ekfcov33 * k53 - ekfcov43 * k54 -
         ekfcov53 * (k55 - 1),
       -ekfcov04 * k50 - ekfcov14 * k51 - ekfcov24 * k52 - ekfcov34 * k53 - ekfcov44 * k54 -
         ekfcov54 * (k55 - 1),
       -ekfcov05 * k50 - ekfcov15 * k51 - ekfcov25 * k52 - ekfcov35 * k53 - ekfcov45 * k54 -
         ekfcov55 * (k55 - 1),
       -ekfcov06 * k50 - ekfcov16 * k51 - ekfcov26 * k52 - ekfcov36 * k53 - ekfcov46 * k54 -
         ekfcov56 * (k55 - 1),
       ekfcov60 - ekfcov00 * k60 - ekfcov10 * k61 - ekfcov20 * k62 - ekfcov30 * k63 -
         ekfcov40 * k64 - ekfcov50 * k65,
       ekfcov61 - ekfcov01 * k60 - ekfcov11 * k61 - ekfcov21 * k62 - ekfcov31 * k63 -
         ekfcov41 * k64 - ekfcov51 * k65,
       ekfcov62 - ekfcov02 * k60 - ekfcov12 * k61 - ekfcov22 * k62 - ekfcov32 * k63 -
         ekfcov42 * k64 - ekfcov52 * k65,
       ekfcov63 - ekfcov03 * k60 - ekfcov13 * k61 - ekfcov23 * k62 - ekfcov33 * k63 -
         ekfcov43 * k64 - ekfcov53 * k65,
       ekfcov64 - ekfcov04 * k60 - ekfcov14 * k61 - ekfcov24 * k62 - ekfcov34 * k63 -
         ekfcov44 * k64 - ekfcov54 * k65,
       ekfcov65 - ekfcov05 * k60 - ekfcov15 * k61 - ekfcov25 * k62 - ekfcov35 * k63 -
         ekfcov45 * k64 - ekfcov55 * k65,
       ekfcov66 - ekfcov06 * k60 - ekfcov16 * k61 - ekfcov26 * k62 - ekfcov36 * k63 -
         ekfcov46 * k64 - ekfcov56 * k65}

    %{state | ekf_state: ekf_state, ekf_cov: ekf_cov, origin: origin}
  end

  def update_from_heading(state, heading_rad) do
    if state.heading_established do
      {ekf_state0, ekf_state1, ekf_state2, ekf_state3, ekf_state4, ekf_state5, ekf_state6} =
        state.ekf_state

      delta_z = ViaUtils.Motion.turn_left_or_right_for_correction(heading_rad - ekf_state6)

      {ekfcov00, ekfcov01, ekfcov02, ekfcov03, ekfcov04, ekfcov05, ekfcov06, ekfcov10, ekfcov11,
       ekfcov12, ekfcov13, ekfcov14, ekfcov15, ekfcov16, ekfcov20, ekfcov21, ekfcov22, ekfcov23,
       ekfcov24, ekfcov25, ekfcov26, ekfcov30, ekfcov31, ekfcov32, ekfcov33, ekfcov34, ekfcov35,
       ekfcov36, ekfcov40, ekfcov41, ekfcov42, ekfcov43, ekfcov44, ekfcov45, ekfcov46, ekfcov50,
       ekfcov51, ekfcov52, ekfcov53, ekfcov54, ekfcov55, ekfcov56, ekfcov60, ekfcov61, ekfcov62,
       ekfcov63, ekfcov64, ekfcov65, ekfcov66} = state.ekf_cov

      {r_heading} = state.r_heading

      mat_div = ekfcov66 + r_heading

      inv_mat = if mat_div != 0, do: 1 / mat_div, else: 0

      k00 = ekfcov06 * inv_mat
      k10 = ekfcov16 * inv_mat
      k20 = ekfcov26 * inv_mat
      k30 = ekfcov36 * inv_mat
      k40 = ekfcov46 * inv_mat
      k50 = ekfcov56 * inv_mat
      k60 = ekfcov66 * inv_mat

      ekf_state = {
        ekf_state0 + delta_z * k00,
        ekf_state1 + delta_z * k10,
        ekf_state2 + delta_z * k20,
        ekf_state3 + delta_z * k30,
        ekf_state4 + delta_z * k40,
        ekf_state5 + delta_z * k50,
        ekf_state6 + delta_z * k60
      }

      ekf_cov =
        {ekfcov00 + ekfcov60 * k00, ekfcov01 + ekfcov61 * k00, ekfcov02 + ekfcov62 * k00,
         ekfcov03 + ekfcov63 * k00, ekfcov04 + ekfcov64 * k00, ekfcov05 + ekfcov65 * k00,
         ekfcov06 + ekfcov66 * k00, ekfcov10 + ekfcov60 * k10, ekfcov11 + ekfcov61 * k10,
         ekfcov12 + ekfcov62 * k10, ekfcov13 + ekfcov63 * k10, ekfcov14 + ekfcov64 * k10,
         ekfcov15 + ekfcov65 * k10, ekfcov16 + ekfcov66 * k10, ekfcov20 + ekfcov60 * k20,
         ekfcov21 + ekfcov61 * k20, ekfcov22 + ekfcov62 * k20, ekfcov23 + ekfcov63 * k20,
         ekfcov24 + ekfcov64 * k20, ekfcov25 + ekfcov65 * k20, ekfcov26 + ekfcov66 * k20,
         ekfcov30 + ekfcov60 * k30, ekfcov31 + ekfcov61 * k30, ekfcov32 + ekfcov62 * k30,
         ekfcov33 + ekfcov63 * k30, ekfcov34 + ekfcov64 * k30, ekfcov35 + ekfcov65 * k30,
         ekfcov36 + ekfcov66 * k30, ekfcov40 + ekfcov60 * k40, ekfcov41 + ekfcov61 * k40,
         ekfcov42 + ekfcov62 * k40, ekfcov43 + ekfcov63 * k40, ekfcov44 + ekfcov64 * k40,
         ekfcov45 + ekfcov65 * k40, ekfcov46 + ekfcov66 * k40, ekfcov50 + ekfcov60 * k50,
         ekfcov51 + ekfcov61 * k50, ekfcov52 + ekfcov62 * k50, ekfcov53 + ekfcov63 * k50,
         ekfcov54 + ekfcov64 * k50, ekfcov55 + ekfcov65 * k50, ekfcov56 + ekfcov66 * k50,
         -ekfcov60 * (k60 - 1), -ekfcov61 * (k60 - 1), -ekfcov62 * (k60 - 1),
         -ekfcov63 * (k60 - 1), -ekfcov64 * (k60 - 1), -ekfcov65 * (k60 - 1),
         -ekfcov66 * (k60 - 1)}

      delta_yaw = delta_z * k60
      imu = ViaEstimation.Imu.Utils.rotate_yaw_rad(state.imu, delta_yaw)
      %{state | imu: imu, ekf_state: ekf_state, ekf_cov: ekf_cov}
    else
      Logger.debug("Established heading at #{ViaUtils.Format.eftb_deg(heading_rad, 2)}")
      ekf_state = state.ekf_state |> Kernel.put_elem(6, heading_rad)

      %{state | ekf_state: ekf_state, heading_established: true}
    end
  end

  # g_prime*ekf_cov
  def mult_77_71(a77, b71) do
    {
      a00,
      a01,
      a02,
      a03,
      a04,
      a05,
      a06,
      a10,
      a11,
      a12,
      a13,
      a14,
      a15,
      a16,
      a20,
      a21,
      a22,
      a23,
      a24,
      a25,
      a26,
      a30,
      a31,
      a32,
      a33,
      a34,
      a35,
      a36,
      a40,
      a41,
      a42,
      a43,
      a44,
      a45,
      a46,
      a50,
      a51,
      a52,
      a53,
      a54,
      a55,
      a56,
      a60,
      a61,
      a62,
      a63,
      a64,
      a65,
      a66
    } = a77

    {b00, b10, b20, b30, b40, b50, b60} = b71

    {
      a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30 + a04 * b40 + a05 * b50 + a06 * b60,
      a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30 + a14 * b40 + a15 * b50 + a16 * b60,
      a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30 + a24 * b40 + a25 * b50 + a26 * b60,
      a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30 + a34 * b40 + a35 * b50 + a36 * b60,
      a40 * b00 + a41 * b10 + a42 * b20 + a43 * b30 + a44 * b40 + a45 * b50 + a46 * b60,
      a50 * b00 + a51 * b10 + a52 * b20 + a53 * b30 + a54 * b40 + a55 * b50 + a56 * b60,
      a60 * b00 + a61 * b10 + a62 * b20 + a63 * b30 + a64 * b40 + a65 * b50 + a66 * b60
    }
  end

  # mat_to_invert
  # h_prime*ekf_cov*h_primeT + r_gps
  def mult_67_77_67t_plus66(a67, b77, r66) do
    {
      a00,
      a01,
      a02,
      a03,
      a04,
      a05,
      a06,
      a10,
      a11,
      a12,
      a13,
      a14,
      a15,
      a16,
      a20,
      a21,
      a22,
      a23,
      a24,
      a25,
      a26,
      a30,
      a31,
      a32,
      a33,
      a34,
      a35,
      a36,
      a40,
      a41,
      a42,
      a43,
      a44,
      a45,
      a46,
      a50,
      a51,
      a52,
      a53,
      a54,
      a55,
      a56
    } = a67

    {
      b00,
      b01,
      b02,
      b03,
      b04,
      b05,
      b06,
      b10,
      b11,
      b12,
      b13,
      b14,
      b15,
      b16,
      b20,
      b21,
      b22,
      b23,
      b24,
      b25,
      b26,
      b30,
      b31,
      b32,
      b33,
      b34,
      b35,
      b36,
      b40,
      b41,
      b42,
      b43,
      b44,
      b45,
      b46,
      b50,
      b51,
      b52,
      b53,
      b54,
      b55,
      b56,
      b60,
      b61,
      b62,
      b63,
      b64,
      b65,
      b66
    } = b77

    {r00, r11, r22, r33, r44, r55} = r66

    c00 = a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30 + a04 * b40 + a05 * b50 + a06 * b60
    c01 = a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31 + a04 * b41 + a05 * b51 + a06 * b61
    c02 = a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32 + a04 * b42 + a05 * b52 + a06 * b62
    c03 = a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33 + a04 * b43 + a05 * b53 + a06 * b63
    c04 = a00 * b04 + a01 * b14 + a02 * b24 + a03 * b34 + a04 * b44 + a05 * b54 + a06 * b64
    c05 = a00 * b05 + a01 * b15 + a02 * b25 + a03 * b35 + a04 * b45 + a05 * b55 + a06 * b65
    c06 = a00 * b06 + a01 * b16 + a02 * b26 + a03 * b36 + a04 * b46 + a05 * b56 + a06 * b66
    c10 = a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30 + a14 * b40 + a15 * b50 + a16 * b60
    c11 = a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31 + a14 * b41 + a15 * b51 + a16 * b61
    c12 = a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32 + a14 * b42 + a15 * b52 + a16 * b62
    c13 = a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33 + a14 * b43 + a15 * b53 + a16 * b63
    c14 = a10 * b04 + a11 * b14 + a12 * b24 + a13 * b34 + a14 * b44 + a15 * b54 + a16 * b64
    c15 = a10 * b05 + a11 * b15 + a12 * b25 + a13 * b35 + a14 * b45 + a15 * b55 + a16 * b65
    c16 = a10 * b06 + a11 * b16 + a12 * b26 + a13 * b36 + a14 * b46 + a15 * b56 + a16 * b66
    c20 = a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30 + a24 * b40 + a25 * b50 + a26 * b60
    c21 = a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31 + a24 * b41 + a25 * b51 + a26 * b61
    c22 = a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32 + a24 * b42 + a25 * b52 + a26 * b62
    c23 = a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33 + a24 * b43 + a25 * b53 + a26 * b63
    c24 = a20 * b04 + a21 * b14 + a22 * b24 + a23 * b34 + a24 * b44 + a25 * b54 + a26 * b64
    c25 = a20 * b05 + a21 * b15 + a22 * b25 + a23 * b35 + a24 * b45 + a25 * b55 + a26 * b65
    c26 = a20 * b06 + a21 * b16 + a22 * b26 + a23 * b36 + a24 * b46 + a25 * b56 + a26 * b66
    c30 = a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30 + a34 * b40 + a35 * b50 + a36 * b60
    c31 = a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31 + a34 * b41 + a35 * b51 + a36 * b61
    c32 = a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32 + a34 * b42 + a35 * b52 + a36 * b62
    c33 = a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33 + a34 * b43 + a35 * b53 + a36 * b63
    c34 = a30 * b04 + a31 * b14 + a32 * b24 + a33 * b34 + a34 * b44 + a35 * b54 + a36 * b64
    c35 = a30 * b05 + a31 * b15 + a32 * b25 + a33 * b35 + a34 * b45 + a35 * b55 + a36 * b65
    c36 = a30 * b06 + a31 * b16 + a32 * b26 + a33 * b36 + a34 * b46 + a35 * b56 + a36 * b66
    c40 = a40 * b00 + a41 * b10 + a42 * b20 + a43 * b30 + a44 * b40 + a45 * b50 + a46 * b60
    c41 = a40 * b01 + a41 * b11 + a42 * b21 + a43 * b31 + a44 * b41 + a45 * b51 + a46 * b61
    c42 = a40 * b02 + a41 * b12 + a42 * b22 + a43 * b32 + a44 * b42 + a45 * b52 + a46 * b62
    c43 = a40 * b03 + a41 * b13 + a42 * b23 + a43 * b33 + a44 * b43 + a45 * b53 + a46 * b63
    c44 = a40 * b04 + a41 * b14 + a42 * b24 + a43 * b34 + a44 * b44 + a45 * b54 + a46 * b64
    c45 = a40 * b05 + a41 * b15 + a42 * b25 + a43 * b35 + a44 * b45 + a45 * b55 + a46 * b65
    c46 = a40 * b06 + a41 * b16 + a42 * b26 + a43 * b36 + a44 * b46 + a45 * b56 + a46 * b66
    c50 = a50 * b00 + a51 * b10 + a52 * b20 + a53 * b30 + a54 * b40 + a55 * b50 + a56 * b60
    c51 = a50 * b01 + a51 * b11 + a52 * b21 + a53 * b31 + a54 * b41 + a55 * b51 + a56 * b61
    c52 = a50 * b02 + a51 * b12 + a52 * b22 + a53 * b32 + a54 * b42 + a55 * b52 + a56 * b62
    c53 = a50 * b03 + a51 * b13 + a52 * b23 + a53 * b33 + a54 * b43 + a55 * b53 + a56 * b63
    c54 = a50 * b04 + a51 * b14 + a52 * b24 + a53 * b34 + a54 * b44 + a55 * b54 + a56 * b64
    c55 = a50 * b05 + a51 * b15 + a52 * b25 + a53 * b35 + a54 * b45 + a55 * b55 + a56 * b65
    c56 = a50 * b06 + a51 * b16 + a52 * b26 + a53 * b36 + a54 * b46 + a55 * b56 + a56 * b66

    {r00 + a00 * c00 + a01 * c01 + a02 * c02 + a03 * c03 + a04 * c04 + a05 * c05 + a06 * c06,
     a10 * c00 + a11 * c01 + a12 * c02 + a13 * c03 + a14 * c04 + a15 * c05 + a16 * c06,
     a20 * c00 + a21 * c01 + a22 * c02 + a23 * c03 + a24 * c04 + a25 * c05 + a26 * c06,
     a30 * c00 + a31 * c01 + a32 * c02 + a33 * c03 + a34 * c04 + a35 * c05 + a36 * c06,
     a40 * c00 + a41 * c01 + a42 * c02 + a43 * c03 + a44 * c04 + a45 * c05 + a46 * c06,
     a50 * c00 + a51 * c01 + a52 * c02 + a53 * c03 + a54 * c04 + a55 * c05 + a56 * c06,
     a00 * c10 + a01 * c11 + a02 * c12 + a03 * c13 + a04 * c14 + a05 * c15 + a06 * c16,
     r11 + a10 * c10 + a11 * c11 + a12 * c12 + a13 * c13 + a14 * c14 + a15 * c15 + a16 * c16,
     a20 * c10 + a21 * c11 + a22 * c12 + a23 * c13 + a24 * c14 + a25 * c15 + a26 * c16,
     a30 * c10 + a31 * c11 + a32 * c12 + a33 * c13 + a34 * c14 + a35 * c15 + a36 * c16,
     a40 * c10 + a41 * c11 + a42 * c12 + a43 * c13 + a44 * c14 + a45 * c15 + a46 * c16,
     a50 * c10 + a51 * c11 + a52 * c12 + a53 * c13 + a54 * c14 + a55 * c15 + a56 * c16,
     a00 * c20 + a01 * c21 + a02 * c22 + a03 * c23 + a04 * c24 + a05 * c25 + a06 * c26,
     a10 * c20 + a11 * c21 + a12 * c22 + a13 * c23 + a14 * c24 + a15 * c25 + a16 * c26,
     r22 + a20 * c20 + a21 * c21 + a22 * c22 + a23 * c23 + a24 * c24 + a25 * c25 + a26 * c26,
     a30 * c20 + a31 * c21 + a32 * c22 + a33 * c23 + a34 * c24 + a35 * c25 + a36 * c26,
     a40 * c20 + a41 * c21 + a42 * c22 + a43 * c23 + a44 * c24 + a45 * c25 + a46 * c26,
     a50 * c20 + a51 * c21 + a52 * c22 + a53 * c23 + a54 * c24 + a55 * c25 + a56 * c26,
     a00 * c30 + a01 * c31 + a02 * c32 + a03 * c33 + a04 * c34 + a05 * c35 + a06 * c36,
     a10 * c30 + a11 * c31 + a12 * c32 + a13 * c33 + a14 * c34 + a15 * c35 + a16 * c36,
     a20 * c30 + a21 * c31 + a22 * c32 + a23 * c33 + a24 * c34 + a25 * c35 + a26 * c36,
     r33 + a30 * c30 + a31 * c31 + a32 * c32 + a33 * c33 + a34 * c34 + a35 * c35 + a36 * c36,
     a40 * c30 + a41 * c31 + a42 * c32 + a43 * c33 + a44 * c34 + a45 * c35 + a46 * c36,
     a50 * c30 + a51 * c31 + a52 * c32 + a53 * c33 + a54 * c34 + a55 * c35 + a56 * c36,
     a00 * c40 + a01 * c41 + a02 * c42 + a03 * c43 + a04 * c44 + a05 * c45 + a06 * c46,
     a10 * c40 + a11 * c41 + a12 * c42 + a13 * c43 + a14 * c44 + a15 * c45 + a16 * c46,
     a20 * c40 + a21 * c41 + a22 * c42 + a23 * c43 + a24 * c44 + a25 * c45 + a26 * c46,
     a30 * c40 + a31 * c41 + a32 * c42 + a33 * c43 + a34 * c44 + a35 * c45 + a36 * c46,
     r44 + a40 * c40 + a41 * c41 + a42 * c42 + a43 * c43 + a44 * c44 + a45 * c45 + a46 * c46,
     a50 * c40 + a51 * c41 + a52 * c42 + a53 * c43 + a54 * c44 + a55 * c45 + a56 * c46,
     a00 * c50 + a01 * c51 + a02 * c52 + a03 * c53 + a04 * c54 + a05 * c55 + a06 * c56,
     a10 * c50 + a11 * c51 + a12 * c52 + a13 * c53 + a14 * c54 + a15 * c55 + a16 * c56,
     a20 * c50 + a21 * c51 + a22 * c52 + a23 * c53 + a24 * c54 + a25 * c55 + a26 * c56,
     a30 * c50 + a31 * c51 + a32 * c52 + a33 * c53 + a34 * c54 + a35 * c55 + a36 * c56,
     a40 * c50 + a41 * c51 + a42 * c52 + a43 * c53 + a44 * c54 + a45 * c55 + a46 * c56,
     r55 + a50 * c50 + a51 * c51 + a52 * c52 + a53 * c53 + a54 * c54 + a55 * c55 + a56 * c56}
  end

  def mult_77_77(a77, b77) do
    {
      a00,
      a01,
      a02,
      a03,
      a04,
      a05,
      a06,
      a10,
      a11,
      a12,
      a13,
      a14,
      a15,
      a16,
      a20,
      a21,
      a22,
      a23,
      a24,
      a25,
      a26,
      a30,
      a31,
      a32,
      a33,
      a34,
      a35,
      a36,
      a40,
      a41,
      a42,
      a43,
      a44,
      a45,
      a46,
      a50,
      a51,
      a52,
      a53,
      a54,
      a55,
      a56,
      a60,
      a61,
      a62,
      a63,
      a64,
      a65,
      a66
    } = a77

    {
      b00,
      b01,
      b02,
      b03,
      b04,
      b05,
      b06,
      b10,
      b11,
      b12,
      b13,
      b14,
      b15,
      b16,
      b20,
      b21,
      b22,
      b23,
      b24,
      b25,
      b26,
      b30,
      b31,
      b32,
      b33,
      b34,
      b35,
      b36,
      b40,
      b41,
      b42,
      b43,
      b44,
      b45,
      b46,
      b50,
      b51,
      b52,
      b53,
      b54,
      b55,
      b56,
      b60,
      b61,
      b62,
      b63,
      b64,
      b65,
      b66
    } = b77

    {a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30 + a04 * b40 + a05 * b50 + a06 * b60,
     a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31 + a04 * b41 + a05 * b51 + a06 * b61,
     a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32 + a04 * b42 + a05 * b52 + a06 * b62,
     a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33 + a04 * b43 + a05 * b53 + a06 * b63,
     a00 * b04 + a01 * b14 + a02 * b24 + a03 * b34 + a04 * b44 + a05 * b54 + a06 * b64,
     a00 * b05 + a01 * b15 + a02 * b25 + a03 * b35 + a04 * b45 + a05 * b55 + a06 * b65,
     a00 * b06 + a01 * b16 + a02 * b26 + a03 * b36 + a04 * b46 + a05 * b56 + a06 * b66,
     a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30 + a14 * b40 + a15 * b50 + a16 * b60,
     a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31 + a14 * b41 + a15 * b51 + a16 * b61,
     a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32 + a14 * b42 + a15 * b52 + a16 * b62,
     a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33 + a14 * b43 + a15 * b53 + a16 * b63,
     a10 * b04 + a11 * b14 + a12 * b24 + a13 * b34 + a14 * b44 + a15 * b54 + a16 * b64,
     a10 * b05 + a11 * b15 + a12 * b25 + a13 * b35 + a14 * b45 + a15 * b55 + a16 * b65,
     a10 * b06 + a11 * b16 + a12 * b26 + a13 * b36 + a14 * b46 + a15 * b56 + a16 * b66,
     a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30 + a24 * b40 + a25 * b50 + a26 * b60,
     a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31 + a24 * b41 + a25 * b51 + a26 * b61,
     a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32 + a24 * b42 + a25 * b52 + a26 * b62,
     a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33 + a24 * b43 + a25 * b53 + a26 * b63,
     a20 * b04 + a21 * b14 + a22 * b24 + a23 * b34 + a24 * b44 + a25 * b54 + a26 * b64,
     a20 * b05 + a21 * b15 + a22 * b25 + a23 * b35 + a24 * b45 + a25 * b55 + a26 * b65,
     a20 * b06 + a21 * b16 + a22 * b26 + a23 * b36 + a24 * b46 + a25 * b56 + a26 * b66,
     a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30 + a34 * b40 + a35 * b50 + a36 * b60,
     a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31 + a34 * b41 + a35 * b51 + a36 * b61,
     a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32 + a34 * b42 + a35 * b52 + a36 * b62,
     a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33 + a34 * b43 + a35 * b53 + a36 * b63,
     a30 * b04 + a31 * b14 + a32 * b24 + a33 * b34 + a34 * b44 + a35 * b54 + a36 * b64,
     a30 * b05 + a31 * b15 + a32 * b25 + a33 * b35 + a34 * b45 + a35 * b55 + a36 * b65,
     a30 * b06 + a31 * b16 + a32 * b26 + a33 * b36 + a34 * b46 + a35 * b56 + a36 * b66,
     a40 * b00 + a41 * b10 + a42 * b20 + a43 * b30 + a44 * b40 + a45 * b50 + a46 * b60,
     a40 * b01 + a41 * b11 + a42 * b21 + a43 * b31 + a44 * b41 + a45 * b51 + a46 * b61,
     a40 * b02 + a41 * b12 + a42 * b22 + a43 * b32 + a44 * b42 + a45 * b52 + a46 * b62,
     a40 * b03 + a41 * b13 + a42 * b23 + a43 * b33 + a44 * b43 + a45 * b53 + a46 * b63,
     a40 * b04 + a41 * b14 + a42 * b24 + a43 * b34 + a44 * b44 + a45 * b54 + a46 * b64,
     a40 * b05 + a41 * b15 + a42 * b25 + a43 * b35 + a44 * b45 + a45 * b55 + a46 * b65,
     a40 * b06 + a41 * b16 + a42 * b26 + a43 * b36 + a44 * b46 + a45 * b56 + a46 * b66,
     a50 * b00 + a51 * b10 + a52 * b20 + a53 * b30 + a54 * b40 + a55 * b50 + a56 * b60,
     a50 * b01 + a51 * b11 + a52 * b21 + a53 * b31 + a54 * b41 + a55 * b51 + a56 * b61,
     a50 * b02 + a51 * b12 + a52 * b22 + a53 * b32 + a54 * b42 + a55 * b52 + a56 * b62,
     a50 * b03 + a51 * b13 + a52 * b23 + a53 * b33 + a54 * b43 + a55 * b53 + a56 * b63,
     a50 * b04 + a51 * b14 + a52 * b24 + a53 * b34 + a54 * b44 + a55 * b54 + a56 * b64,
     a50 * b05 + a51 * b15 + a52 * b25 + a53 * b35 + a54 * b45 + a55 * b55 + a56 * b65,
     a50 * b06 + a51 * b16 + a52 * b26 + a53 * b36 + a54 * b46 + a55 * b56 + a56 * b66,
     a60 * b00 + a61 * b10 + a62 * b20 + a63 * b30 + a64 * b40 + a65 * b50 + a66 * b60,
     a60 * b01 + a61 * b11 + a62 * b21 + a63 * b31 + a64 * b41 + a65 * b51 + a66 * b61,
     a60 * b02 + a61 * b12 + a62 * b22 + a63 * b32 + a64 * b42 + a65 * b52 + a66 * b62,
     a60 * b03 + a61 * b13 + a62 * b23 + a63 * b33 + a64 * b43 + a65 * b53 + a66 * b63,
     a60 * b04 + a61 * b14 + a62 * b24 + a63 * b34 + a64 * b44 + a65 * b54 + a66 * b64,
     a60 * b05 + a61 * b15 + a62 * b25 + a63 * b35 + a64 * b45 + a65 * b55 + a66 * b65,
     a60 * b06 + a61 * b16 + a62 * b26 + a63 * b36 + a64 * b46 + a65 * b56 + a66 * b66}
  end

  def mult_77_77t_plus_q77diag(a77, b77, q77) do
    {
      a00,
      a01,
      a02,
      a03,
      a04,
      a05,
      a06,
      a10,
      a11,
      a12,
      a13,
      a14,
      a15,
      a16,
      a20,
      a21,
      a22,
      a23,
      a24,
      a25,
      a26,
      a30,
      a31,
      a32,
      a33,
      a34,
      a35,
      a36,
      a40,
      a41,
      a42,
      a43,
      a44,
      a45,
      a46,
      a50,
      a51,
      a52,
      a53,
      a54,
      a55,
      a56,
      a60,
      a61,
      a62,
      a63,
      a64,
      a65,
      a66
    } = a77

    {
      b00,
      b01,
      b02,
      b03,
      b04,
      b05,
      b06,
      b10,
      b11,
      b12,
      b13,
      b14,
      b15,
      b16,
      b20,
      b21,
      b22,
      b23,
      b24,
      b25,
      b26,
      b30,
      b31,
      b32,
      b33,
      b34,
      b35,
      b36,
      b40,
      b41,
      b42,
      b43,
      b44,
      b45,
      b46,
      b50,
      b51,
      b52,
      b53,
      b54,
      b55,
      b56,
      b60,
      b61,
      b62,
      b63,
      b64,
      b65,
      b66
    } = b77

    {q00, q11, q22, q33, q44, q55, q66} = q77

    {q00 + a00 * b00 + a01 * b01 + a02 * b02 + a03 * b03 + a04 * b04 + a05 * b05 + a06 * b06,
     a00 * b10 + a01 * b11 + a02 * b12 + a03 * b13 + a04 * b14 + a05 * b15 + a06 * b16,
     a00 * b20 + a01 * b21 + a02 * b22 + a03 * b23 + a04 * b24 + a05 * b25 + a06 * b26,
     a00 * b30 + a01 * b31 + a02 * b32 + a03 * b33 + a04 * b34 + a05 * b35 + a06 * b36,
     a00 * b40 + a01 * b41 + a02 * b42 + a03 * b43 + a04 * b44 + a05 * b45 + a06 * b46,
     a00 * b50 + a01 * b51 + a02 * b52 + a03 * b53 + a04 * b54 + a05 * b55 + a06 * b56,
     a00 * b60 + a01 * b61 + a02 * b62 + a03 * b63 + a04 * b64 + a05 * b65 + a06 * b66,
     a10 * b00 + a11 * b01 + a12 * b02 + a13 * b03 + a14 * b04 + a15 * b05 + a16 * b06,
     q11 + a10 * b10 + a11 * b11 + a12 * b12 + a13 * b13 + a14 * b14 + a15 * b15 + a16 * b16,
     a10 * b20 + a11 * b21 + a12 * b22 + a13 * b23 + a14 * b24 + a15 * b25 + a16 * b26,
     a10 * b30 + a11 * b31 + a12 * b32 + a13 * b33 + a14 * b34 + a15 * b35 + a16 * b36,
     a10 * b40 + a11 * b41 + a12 * b42 + a13 * b43 + a14 * b44 + a15 * b45 + a16 * b46,
     a10 * b50 + a11 * b51 + a12 * b52 + a13 * b53 + a14 * b54 + a15 * b55 + a16 * b56,
     a10 * b60 + a11 * b61 + a12 * b62 + a13 * b63 + a14 * b64 + a15 * b65 + a16 * b66,
     a20 * b00 + a21 * b01 + a22 * b02 + a23 * b03 + a24 * b04 + a25 * b05 + a26 * b06,
     a20 * b10 + a21 * b11 + a22 * b12 + a23 * b13 + a24 * b14 + a25 * b15 + a26 * b16,
     q22 + a20 * b20 + a21 * b21 + a22 * b22 + a23 * b23 + a24 * b24 + a25 * b25 + a26 * b26,
     a20 * b30 + a21 * b31 + a22 * b32 + a23 * b33 + a24 * b34 + a25 * b35 + a26 * b36,
     a20 * b40 + a21 * b41 + a22 * b42 + a23 * b43 + a24 * b44 + a25 * b45 + a26 * b46,
     a20 * b50 + a21 * b51 + a22 * b52 + a23 * b53 + a24 * b54 + a25 * b55 + a26 * b56,
     a20 * b60 + a21 * b61 + a22 * b62 + a23 * b63 + a24 * b64 + a25 * b65 + a26 * b66,
     a30 * b00 + a31 * b01 + a32 * b02 + a33 * b03 + a34 * b04 + a35 * b05 + a36 * b06,
     a30 * b10 + a31 * b11 + a32 * b12 + a33 * b13 + a34 * b14 + a35 * b15 + a36 * b16,
     a30 * b20 + a31 * b21 + a32 * b22 + a33 * b23 + a34 * b24 + a35 * b25 + a36 * b26,
     q33 + a30 * b30 + a31 * b31 + a32 * b32 + a33 * b33 + a34 * b34 + a35 * b35 + a36 * b36,
     a30 * b40 + a31 * b41 + a32 * b42 + a33 * b43 + a34 * b44 + a35 * b45 + a36 * b46,
     a30 * b50 + a31 * b51 + a32 * b52 + a33 * b53 + a34 * b54 + a35 * b55 + a36 * b56,
     a30 * b60 + a31 * b61 + a32 * b62 + a33 * b63 + a34 * b64 + a35 * b65 + a36 * b66,
     a40 * b00 + a41 * b01 + a42 * b02 + a43 * b03 + a44 * b04 + a45 * b05 + a46 * b06,
     a40 * b10 + a41 * b11 + a42 * b12 + a43 * b13 + a44 * b14 + a45 * b15 + a46 * b16,
     a40 * b20 + a41 * b21 + a42 * b22 + a43 * b23 + a44 * b24 + a45 * b25 + a46 * b26,
     a40 * b30 + a41 * b31 + a42 * b32 + a43 * b33 + a44 * b34 + a45 * b35 + a46 * b36,
     q44 + a40 * b40 + a41 * b41 + a42 * b42 + a43 * b43 + a44 * b44 + a45 * b45 + a46 * b46,
     a40 * b50 + a41 * b51 + a42 * b52 + a43 * b53 + a44 * b54 + a45 * b55 + a46 * b56,
     a40 * b60 + a41 * b61 + a42 * b62 + a43 * b63 + a44 * b64 + a45 * b65 + a46 * b66,
     a50 * b00 + a51 * b01 + a52 * b02 + a53 * b03 + a54 * b04 + a55 * b05 + a56 * b06,
     a50 * b10 + a51 * b11 + a52 * b12 + a53 * b13 + a54 * b14 + a55 * b15 + a56 * b16,
     a50 * b20 + a51 * b21 + a52 * b22 + a53 * b23 + a54 * b24 + a55 * b25 + a56 * b26,
     a50 * b30 + a51 * b31 + a52 * b32 + a53 * b33 + a54 * b34 + a55 * b35 + a56 * b36,
     a50 * b40 + a51 * b41 + a52 * b42 + a53 * b43 + a54 * b44 + a55 * b45 + a56 * b46,
     q55 + a50 * b50 + a51 * b51 + a52 * b52 + a53 * b53 + a54 * b54 + a55 * b55 + a56 * b56,
     a50 * b60 + a51 * b61 + a52 * b62 + a53 * b63 + a54 * b64 + a55 * b65 + a56 * b66,
     a60 * b00 + a61 * b01 + a62 * b02 + a63 * b03 + a64 * b04 + a65 * b05 + a66 * b06,
     a60 * b10 + a61 * b11 + a62 * b12 + a63 * b13 + a64 * b14 + a65 * b15 + a66 * b16,
     a60 * b20 + a61 * b21 + a62 * b22 + a63 * b23 + a64 * b24 + a65 * b25 + a66 * b26,
     a60 * b30 + a61 * b31 + a62 * b32 + a63 * b33 + a64 * b34 + a65 * b35 + a66 * b36,
     a60 * b40 + a61 * b41 + a62 * b42 + a63 * b43 + a64 * b44 + a65 * b45 + a66 * b46,
     a60 * b50 + a61 * b51 + a62 * b52 + a63 * b53 + a64 * b54 + a65 * b55 + a66 * b56,
     q66 + a60 * b60 + a61 * b61 + a62 * b62 + a63 * b63 + a64 * b64 + a65 * b65 + a66 * b66}
  end

  @spec to_2d_list(list(), integer(), integer()) :: list()
  def to_2d_list(matrix, rows, columns) do
    {_, mat_2d_list} =
      Enum.reduce(1..rows, {matrix, []}, fn _row_num, {matrix_rem, acc} ->
        {row, matrix_rem} = Enum.split(matrix_rem, columns)
        {matrix_rem, acc ++ [row]}
      end)

    Logger.debug(inspect(mat_2d_list))
    mat_2d_list
  end

  @spec generate_ekf_cov(list()) :: tuple()
  def generate_ekf_cov(config) do
    init_std_devs =
      Keyword.fetch!(config, :init_std_devs)
      |> Enum.with_index()

    ekf_cov_list =
      Enum.reduce(init_std_devs, [], fn {std_dev, index}, acc ->
        row =
          [0, 0, 0, 0, 0, 0, 0]
          |> List.replace_at(index, std_dev * std_dev)

        acc ++ row
      end)

    List.to_tuple(ekf_cov_list)
  end

  @spec generate_r_gps(list()) :: tuple()
  def generate_r_gps(config) do
    keys = [
      :gpspos_xy_std,
      :gpspos_xy_std,
      :gpspos_z_std,
      :gpsvel_xy_std,
      :gpsvel_xy_std,
      :gpsvel_z_std
    ]

    Enum.map(keys, fn key ->
      value = Keyword.fetch!(config, key)
      value * value
    end)
    |> List.to_tuple()
  end

  @spec generate_r_heading(list()) :: tuple()
  def generate_r_heading(config) do
    value = Keyword.fetch!(config, :gpsyaw_std)
    {value * value}
  end

  @spec generate_q(list()) :: tuple()
  def generate_q(config) do
    keys = [
      :qpos_xy_std,
      :qpos_xy_std,
      :qpos_z_std,
      :qvel_xy_std,
      :qvel_xy_std,
      :qvel_z_std,
      :qyaw_std
    ]

    expected_imu_dt_s = Keyword.fetch!(config, :expected_imu_dt_s)

    Enum.map(keys, fn key ->
      value = Keyword.fetch!(config, key)
      value * value * expected_imu_dt_s
    end)
    |> List.to_tuple()
  end

  @spec position_rrm(struct()) :: struct()
  def position_rrm(state) do
    if is_nil(state.origin) do
      nil
    else
      {latitude_rad, longitude_rad, position_down_m, _, _, _, _} = state.ekf_state

      ViaUtils.Location.location_from_point_with_dx_dy(state.origin, latitude_rad, longitude_rad)
      |> Map.put(SVN.altitude_m(), -position_down_m)
    end
  end

  @spec velocity_mps(struct()) :: map()
  def velocity_mps(state) do
    {_, _, _, v_north_mps, v_east_mps, v_down_mps} = state.ekf_state

    %{
      SVN.v_north_mps() => v_north_mps,
      SVN.v_east_mps() => v_east_mps,
      SVN.v_down_mps() => v_down_mps
    }
  end

  @spec position_rrm_velocity_mps(struct()) :: tuple()
  def position_rrm_velocity_mps(state) do
    {latitude_rad, longitude_rad, position_down_m, v_north_mps, v_east_mps, v_down_mps, _} =
      state.ekf_state

    position_rrm =
      if is_nil(state.origin) do
        nil
      else
        ViaUtils.Location.location_from_point_with_dx_dy(
          state.origin,
          latitude_rad,
          longitude_rad
        )
        |> Map.put(SVN.altitude_m(), -position_down_m)
      end

    velocity_mps = %{
      SVN.v_north_mps() => v_north_mps,
      SVN.v_east_mps() => v_east_mps,
      SVN.v_down_mps() => v_down_mps
    }

    {position_rrm, velocity_mps}
  end
end
