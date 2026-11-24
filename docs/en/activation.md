# License Activation

How to activate your product key, change it, and what to do when activation
fails.

## Where the key comes from

The vendor sends you a product key (a short string). One key is bound to one
machine: the first activation stamps the key with your machine ID, and the
same key then only validates on that machine (contact the vendor to unbind /
re-issue when changing hardware).

## Activating

1. Start the launcher. On an unlicensed install every control is greyed out
   and the top bar shows `License required to use AC Simulator Suite.` with
   one active button: **[Activate]**.
2. Click **Activate**, paste the key, confirm.
3. On success everything unlocks and the top bar permanently shows
   `License OK (<expiry date>)`.

To move the license to another machine, ask the vendor to unbind the key
first.

## Changing or renewing a key

When licensed, the button becomes **[Change key]** — click it to enter a new
key (renewal or replacement).

## Diagnostics

Everything about license checks is logged to
`ac_simulator_suite\logs\license_dbg.log` (requests and responses). If
activation fails:

1. Check the key for typos (copy-paste it).
2. Check network / firewall to the license server.
3. Look at the last lines of `license_dbg.log` — it names the exact HTTP
   error.
4. If the key was already used on another machine, the server refuses it —
   contact the vendor to unbind.

Related: [launcher.md](launcher.md) (launcher reference),
[troubleshooting.md](troubleshooting.md).
