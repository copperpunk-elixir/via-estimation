defmodule ViaMatrixSevenState.Mult6777Test do
  use ExUnit.Case
  require Logger

  test "Mult 6x7 and 7x7 matrix" do
    m67 =
      Matrex.new([
        [0.41489, 0.26612, 0.659, 0.67385, 0.51434, 0.35346, 0.64266],
        [0.92771, 0.30287, 0.9397, 0.43429, 0.39596, 0.58709, 0.69935],
        [0.70419, 0.62002, 0.02622, 0.50706, 0.80481, 0.88393, 0.07238],
        [0.38584, 0.41295, 0.78461, 0.56844, 0.67784, 0.17835, 0.36335],
        [0.14225, 0.78259, 0.89036, 0.55714, 0.04871, 0.54936, 0.231],
        [0.56305, 0.90283, 0.87366, 0.49076, 0.20569, 0.81336, 0.92505]
      ])

    m77 =
      Matrex.new([
        [0.75848, 0.21514, 0.38555, 0.0618, 0.49464, 0.15186, 0.53084],
        [0.76245, 0.39623, 0.67131, 0.14691, 0.60158, 0.73588, 0.68151],
        [0.61232, 0.19951, 0.21049, 0.7321, 0.95048, 0.66188, 0.13199],
        [0.25896, 0.24823, 0.69446, 0.96881, 0.41336, 0.94941, 0.29704],
        [0.50658, 0.09309, 0.64739, 0.26506, 0.30823, 0.03293, 0.32687],
        [0.80287, 0.18479, 0.85771, 0.56532, 0.58102, 0.52902, 0.71223],
        [0.1826, 0.2649, 0.39374, 0.79491, 0.46441, 0.60424, 0.52701]
      ])

    num_ops = 10
    start_time_matrex = :erlang.monotonic_time(:nanosecond)
        Enum.each(1..num_ops-1, fn _x ->

    m_matrex = Matrex.dot(m67, m77)
        end)
    m_matrex = Matrex.dot(m67, m77)
    end_time_matrex = :erlang.monotonic_time(:nanosecond)
    m67_list = ViaEstimation.Matrix.matrex_to_1d_list(m67, 6, 7)
    m77_list = ViaEstimation.Matrix.matrex_to_1d_list(m77, 7, 7)
    start_time_via = :erlang.monotonic_time(:nanosecond)
    Enum.each(1..num_ops-1, fn _x ->
    m_via = ViaEstimation.Matrix.mult_67_77(m67_list, m77_list)
    end)
    m_via = ViaEstimation.Matrix.mult_67_77(m67_list, m77_list)
    end_time_via = :erlang.monotonic_time(:nanosecond)
    m_via_matrex = ViaEstimation.Matrix.list_to_matrex(m_via, 6, 7)
    IO.puts(inspect(m_matrex))
    IO.puts(inspect(m_via_matrex))
    IO.puts("dt matrex: #{ViaUtils.Format.eftb((end_time_matrex - start_time_matrex) * (1.0e-6)/num_ops, 3)} ms")
    IO.puts("dt via: #{ViaUtils.Format.eftb((end_time_via - start_time_via) * (1.0e-6)/num_ops, 3)} ms")
    Enum.each(1..6, fn i ->
      Enum.each(1..7, fn j ->
        assert_in_delta(m_matrex[i][j], m_via_matrex[i][j], 0.0001)
      end)
    end)
  end
end
