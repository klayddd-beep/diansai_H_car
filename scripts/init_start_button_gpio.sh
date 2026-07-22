#!/usr/bin/env bash
set -euo pipefail

config=/etc/default/diansai-fire
if [[ -r "$config" ]]; then
  # Optional root-owned overrides for BUTTON_WPI and BUTTON_GPIO.
  # shellcheck disable=SC1090
  source "$config"
fi

button_wpi="${BUTTON_WPI:-9}"
button_gpio="${BUTTON_GPIO:-35}"
gpio_value="/sys/class/gpio/gpio${button_gpio}/value"

if [[ ! -x /usr/bin/gpio ]]; then
  echo "missing /usr/bin/gpio (install wiringOP first)" >&2
  exit 1
fi

/usr/bin/gpio mode "$button_wpi" in
/usr/bin/gpio mode "$button_wpi" up

if [[ ! -d "/sys/class/gpio/gpio${button_gpio}" ]]; then
  printf '%s' "$button_gpio" > /sys/class/gpio/export
fi

for _ in {1..50}; do
  [[ -e "$gpio_value" ]] && break
  sleep 0.02
done
if [[ ! -e "$gpio_value" ]]; then
  echo "GPIO value file did not appear: $gpio_value" >&2
  exit 1
fi

printf 'in' > "/sys/class/gpio/gpio${button_gpio}/direction"
chmod a+r "$gpio_value"

level=$(<"$gpio_value")
echo "start button ready: wPi ${button_wpi}, Linux GPIO ${button_gpio}, released level=${level}"
