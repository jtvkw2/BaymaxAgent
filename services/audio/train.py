import os
import random

# Trainer: Where the ✨️ happens.
# TrainingArgs: Defines the set of arguments of the Trainer.
from trainer import Trainer, TrainerArgs

# GlowTTSConfig: all model related values for training, validating and testing.
from TTS.tts.configs.vits_config import VitsConfig

# BaseDatasetConfig: defines name, formatter and path of the dataset.
from TTS.tts.configs.shared_configs import BaseDatasetConfig, BaseAudioConfig
from TTS.tts.datasets import load_tts_samples
from TTS.tts.models.vits import Vits
from TTS.tts.utils.text.tokenizer import TTSTokenizer
from TTS.utils.audio import AudioProcessor

def formatter(root_path, manifest_file, **kwargs):  # pylint: disable=unused-argument
    """Assumes each line as ```<filename>|<transcription>```
    """
    txt_file = os.path.join(root_path, manifest_file)
    items = []
    speaker_name = "Baymax"
    with open(txt_file, "r", encoding="utf-8") as ttf:
        for line in ttf:
            cols = line.split("|")
            # explicitly add '.wav' here
            wav_file = os.path.join(root_path, "wavs", cols[0] + '.wav')
            text = cols[1]
            items.append({"text": text, "audio_file": wav_file,
                         "speaker_name": speaker_name, "root_path": root_path})
    return items


def split_data(samples, eval_split_size):
    # shuffle the samples
    random.shuffle(samples)

    # calculate split index
    split_idx = int(len(samples) * (1 - eval_split_size))

    # split the samples
    train_samples = samples[:split_idx]
    eval_samples = samples[split_idx:]

    return train_samples, eval_samples


if __name__ == '__main__':
    # we use the same path as this script as our training folder.
    output_path = os.path.dirname(os.path.abspath(__file__))

    # DEFINE DATASET CONFIG
    # Set LJSpeech as our target dataset and define its path.
    # You can also use a simple Dict to define the dataset and pass it to your custom formatter.
    dataset_config = BaseDatasetConfig(
        formatter="ljspeech", meta_file_train="metadata.txt", path=os.path.join(output_path)
    )
    audio_config = BaseAudioConfig(sample_rate=44100, resample=True, do_trim_silence=True)
    # INITIALIZE THE TRAINING CONFIGURATION
    # Configure the model. Every config class inherits the BaseTTSConfig.
    config = VitsConfig(
        batch_size=16,  # Decreased batch size
        eval_batch_size=8,  # Decreased evaluation batch size
        num_loader_workers=2,
        num_eval_loader_workers=2,
        run_eval=True,
        test_delay_epochs=1,  # Evaluate after every epoch
        epochs=2000,  # Increased number of epochs
        text_cleaner="english_cleaners",
        use_phonemes=True,
        phoneme_language="en-us",
        phoneme_cache_path=os.path.join(output_path, "phoneme_cache"),
        print_step=25,
        print_eval=False,
        mixed_precision=True,
        output_path=output_path,
        datasets=[dataset_config],
        eval_split_size=0.2,
    )

    # INITIALIZE THE AUDIO PROCESSOR
    # Audio processor is used for feature extraction and audio I/O.
    # It mainly serves to the dataloader and the training loggers.
    ap = AudioProcessor.init_from_config(audio_config)

    # INITIALIZE THE TOKENIZER
    # Tokenizer is used to convert text to sequences of token IDs.
    # If characters are not defined in the config, default characters are passed to the config
    tokenizer, config = TTSTokenizer.init_from_config(config)

    all_samples = load_tts_samples(
        dataset_config,
        eval_split=False,
        formatter=formatter,
    )

    train_samples, eval_samples = split_data(all_samples[0], config.eval_split_size)

    # INITIALIZE THE MODEL
    # Models take a config object and a speaker manager as input
    # Config defines the details of the model like the number of layers, the size of the embedding, etc.
    # Speaker manager is used by multi-speaker models.
    model = Vits(config, ap, tokenizer, speaker_manager=None)

    # INITIALIZE THE TRAINER
    # Trainer provides a generic API to train all the 🐸TTS models with all its perks like mixed-precision training,
    # distributed training, etc.
    trainer = Trainer(
        TrainerArgs(), config, output_path, model=model, train_samples=train_samples, eval_samples=eval_samples
    )

    # AND... 3,2,1... 🚀
    trainer.fit()