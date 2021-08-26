defmodule ViaEstimation.Matrix.Mult6777Test do
  use ExUnit.Case
  require Logger

  test "Mult 6x7 and 7x7 matrix" do
    a_rows = 6
    a_columns = 7
    b_rows = 7
    b_columns = 7

    {m_matrex, m_via_matrex} =
      ViaEstimation.Matrix.Generic.test_mult(a_rows, a_columns, b_rows, b_columns, 100)

    Enum.each(1..a_rows, fn i ->
      Enum.each(1..b_columns, fn j ->
        assert_in_delta(Matrex.at(m_matrex, i, j), Matrex.at(m_via_matrex, i, j), 0.0001)
      end)
    end)
  end
end
