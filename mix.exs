defmodule ViaEstimation.MixProject do
  use Mix.Project

  @version "0.1.1"
  @source_url "https://github.com/copperpunk-elixir/via-estimation"

  def project do
    [
      app: :via_estimation,
      version: @version,
      elixir: "~> 1.12",
      description: description(),
      package: package(),
      source_url: @source_url,
      docs: docs(),
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp description do
    "Estimation algorithms (INS, IMU) for use with the Via autopilot."
  end

  defp package do
    %{
      licenses: ["GPL-3.0"],
      links: %{"Github" => @source_url}
    }
  end

  defp docs do
    [
      extras: ["README.md"],
      main: "readme",
      source_ref: "v#{@version}",
      source_url: @source_url
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:ex_doc, "~> 0.24", only: :dev, runtime: false},
      {:ring_logger, "~> 0.8.1"},
      {:via_utils, path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-utils"}# git: "https://github.com/copperpunk-elixir/via-utils.git", tag: "v0.1.4-alpha"}
    ]
  end
end
