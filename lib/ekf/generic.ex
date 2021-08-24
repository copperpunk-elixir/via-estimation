defmodule ViaEstimation.Matrix.Generic do
  def test_mult(a_rows, a_cols, b_rows, b_cols, num_ops) do
    func = "mult_#{a_rows}#{a_cols}_#{b_rows}#{b_cols}" |> String.to_atom()
    c_rows = a_rows
    c_columns = b_cols
    mA = Matrex.random(a_rows, a_cols)
    mB = Matrex.random(b_rows, b_cols)
    start_time_matrex = :erlang.monotonic_time(:nanosecond)

    m_matrex = Matrex.dot(mA, mB)

    if num_ops > 1 do
      Enum.each(1..(num_ops - 1), fn _x ->
        Matrex.dot(mA, mB)
      end)
    end

    end_time_matrex = :erlang.monotonic_time(:nanosecond)

    mA_list = Matrex.to_list(mA)
    mB_list = Matrex.to_list(mB)
    mA_tuple = List.to_tuple(mA_list)
    mB_tuple = List.to_tuple(mB_list)
    start_time_via = :erlang.monotonic_time(:nanosecond)

    m_via = apply(ViaEstimation.Matrix, func, [mA_tuple, mB_tuple])

    if num_ops > 1 do
      Enum.each(1..(num_ops - 1), fn _x ->
        apply(ViaEstimation.Matrix, func, [mA_tuple, mB_tuple])
      end)
    end

    end_time_via = :erlang.monotonic_time(:nanosecond)
    m_via_matrex = ViaEstimation.Matrix.list_to_matrex(Tuple.to_list(m_via), c_rows, c_columns)
    IO.puts(inspect(m_matrex))
    IO.puts(inspect(m_via_matrex))

    IO.puts(
      "dt matrex: #{ViaUtils.Format.eftb((end_time_matrex - start_time_matrex) * 1.0e-6 / num_ops, 3)} ms"
    )

    IO.puts(
      "dt via: #{ViaUtils.Format.eftb((end_time_via - start_time_via) * 1.0e-6 / num_ops, 3)} ms"
    )

    {m_matrex, m_via_matrex}
  end
end
