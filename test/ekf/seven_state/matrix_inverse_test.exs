defmodule ViaEstimation.Ekf.SevenState.MatrixInverseTest do
  use ExUnit.Case
  require Logger

  test "invert matrix" do
    a =
      Matrex.new([
        [0.44129651, 0.50729807, 0.44779978, 0.69561188, 0.41901522, 0.2086824],
        [0.70760578, 0.50942716, 0.34224572, 0.01296789, 0.17421562, 0.45051432],
        [0.0252048, 0.1640584, 0.51895142, 0.70362294, 0.79260722, 0.03630458],
        [0.15994082, 0.48996116, 0.99190012, 0.86504746, 0.5725891, 0.78010361],
        [0.98859098, 0.26283676, 0.50195848, 0.30203132, 0.13981192, 0.2716063],
        [0.21668906, 0.99407819, 0.33197042, 0.57297811, 0.06925459, 0.11680278]
      ])

    start_time = :erlang.monotonic_time(:nanosecond)
    a_inv = ViaEstimation.Ekf.SevenState.inv_66(a)
    end_time = :erlang.monotonic_time(:nanosecond)
    # IO.puts(inspect(a_inv))
    IO.puts("dt: #{ViaUtils.Format.eftb((end_time - start_time) * 1.0e-6, 3)} ms")

    a_sol =
      Matrex.new([
        [1.20816788, 0.18319052, -0.35051342, -0.52441256, 0.55333766, -0.54041471],
        [-0.86425526, 0.90618071, 0.47310112, -0.3242323, -0.4837607, 1.19225739],
        [-6.45047672, -0.96255312, 2.31444911, 0.83532015, 2.88580098, 2.22838733],
        [3.96310159, -1.4989107, -1.7389489, 0.1929519, -0.47791288, -0.93606786],
        [0.68066399, 1.72091666, 1.3038623, -0.67151377, -1.29098315, -0.77212969],
        [3.60264071, 1.01616544, -2.1968186, 0.8098552, -2.0013796, -1.86668402]
      ])

    Enum.each(1..6, fn i ->
      Enum.each(1..6, fn j ->
        assert_in_delta(a_inv[i][j], a_sol[i][j], 0.0001)
      end)
    end)
  end
end
