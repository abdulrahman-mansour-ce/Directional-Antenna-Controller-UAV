import numpy as np
from gnuradio import gr

class blk(gr.decim_block):
    def __init__(
        self,
        samp_rate=4e6,
        f_tone=200e3,
        fft_size=4096,
        carrier_bin_half_width=3,
        carried_bandwidth_hz=800e3,
        k_dbfs_to_dbm=0.0
    ):
        gr.decim_block.__init__(
            self,
            name="Carrier and Carried Power",
            in_sig=[np.complex64],
            out_sig=[np.float32, np.float32],
            decim=int(fft_size)
        )

        self.samp_rate = float(samp_rate)
        self.f_tone = float(f_tone)
        self.fft_size = int(fft_size)
        self.carrier_bin_half_width = int(carrier_bin_half_width)
        self.carried_bandwidth_hz = float(carried_bandwidth_hz)
        self.k_dbfs_to_dbm = float(k_dbfs_to_dbm)

        self.window = np.blackman(self.fft_size).astype(np.float32)
        self.window_correction = np.sum(self.window) / 2.0

        self.freqs = np.fft.fftshift(
            np.fft.fftfreq(self.fft_size, d=1.0 / self.samp_rate)
        )

    def work(self, input_items, output_items):
        x = input_items[0]

        carrier_out = output_items[0]
        carried_out = output_items[1]

        nout = min(len(carrier_out), len(carried_out))

        for i in range(nout):
            start = i * self.fft_size
            stop = start + self.fft_size
            frame = x[start:stop]

            if len(frame) < self.fft_size:
                carrier_out[i] = -999.0
                carried_out[i] = -999.0
                continue

            frame = frame.astype(np.complex64)

            # Remove DC only because our wanted signal is shifted to 200 kHz
            if abs(self.f_tone) > 1.0:
                frame = frame - np.mean(frame)

            X = np.fft.fftshift(np.fft.fft(frame * self.window))
            X = X / self.window_correction

            # Find the tone/carrier location
            tone_idx_pos = int(np.argmin(np.abs(self.freqs - self.f_tone)))
            tone_idx_neg = int(np.argmin(np.abs(self.freqs + self.f_tone)))

            if np.abs(X[tone_idx_neg]) > np.abs(X[tone_idx_pos]):
                tone_idx = tone_idx_neg
            else:
                tone_idx = tone_idx_pos

            tone_freq = self.freqs[tone_idx]

            # Carrier / tone power: narrow bins around tone
            hw = self.carrier_bin_half_width
            lo = max(0, tone_idx - hw)
            hi = min(len(X), tone_idx + hw + 1)

            carrier_power_lin = np.sum(np.abs(X[lo:hi]) ** 2)
            carrier_power_dbfs = 10.0 * np.log10(carrier_power_lin + 1e-12)
            carrier_power_dbm = carrier_power_dbfs + self.k_dbfs_to_dbm

            # Carried / signal-band power: wider bandwidth around tone
            half_bw = self.carried_bandwidth_hz / 2.0
            band_mask = np.abs(self.freqs - tone_freq) <= half_bw

            total_band_power_lin = np.sum(np.abs(X[band_mask]) ** 2)

            # Subtract narrow carrier power so carried power is mainly signal-band energy
            carried_power_lin = max(total_band_power_lin - carrier_power_lin, 1e-12)
            carried_power_dbfs = 10.0 * np.log10(carried_power_lin + 1e-12)
            carried_power_dbm = carried_power_dbfs + self.k_dbfs_to_dbm

            carrier_out[i] = np.float32(carrier_power_dbm)
            carried_out[i] = np.float32(carried_power_dbm)

        return nout