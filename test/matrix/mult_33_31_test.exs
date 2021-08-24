defmodule ViaMatrixSevenState.Mult3331Test do
  use ExUnit.Case
  require Logger

  test "Mult 3x3 and 3x1 matrix" do
    a_rows = 3
    a_columns = 3
    b_rows = 3
    b_columns = 1

    {m_matrex, m_via_matrex} =
      ViaEstimation.Matrix.Generic.test_mult(a_rows, a_columns, b_rows, b_columns, 100)

    Enum.each(1..a_rows, fn i ->
      Enum.each(1..b_columns, fn j ->
        assert_in_delta(Matrex.at(m_matrex, i, j), Matrex.at(m_via_matrex, i, j), 0.0001)
      end)
    end)
  end
end
