# -*- coding: utf-8 -*-
#
# Copyright (C) 2017-2025 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import logging
import os
import shutil
import tempfile
import wave
from collections import defaultdict
from functools import partial

import azure.cognitiveservices.speech as speechsdk
import numpy as np

from ttsserver.action_parser import ActionParser
from ttsserver.ttsbase import TTSBase, is_ssml, wave_to_mp3

logger = logging.getLogger("hr.ttsserver.voices.azure")


class AzureTTS(TTSBase):
    VENDOR = "azure"

    # Sets the synthesis voice name.
    # The full list of supported voices can be found here:
    # https://docs.microsoft.com/azure/cognitive-services/speech-service/language-support#text-to-speech
    # https://docs.microsoft.com/en-us/azure/cognitive-services/speech-service/language-support#prebuilt-neural-voices
    ALL = [
        "af-ZA-AdriNeural",
        "af-ZA-WillemNeural",
        "am-ET-AmehaNeural",
        "am-ET-MekdesNeural",
        "ar-AE-FatimaNeural",
        "ar-AE-HamdanNeural",
        "ar-BH-AliNeural",
        "ar-BH-LailaNeural",
        "ar-DZ-AminaNeural",
        "ar-DZ-IsmaelNeural",
        "ar-EG-SalmaNeural",
        "ar-EG-ShakirNeural",
        "ar-IQ-BasselNeural",
        "ar-IQ-RanaNeural",
        "ar-JO-SanaNeural",
        "ar-JO-TaimNeural",
        "ar-KW-FahedNeural",
        "ar-KW-NouraNeural",
        "ar-LB-LaylaNeural",
        "ar-LB-RamiNeural",
        "ar-LY-ImanNeural",
        "ar-LY-OmarNeural",
        "ar-MA-JamalNeural",
        "ar-MA-MounaNeural",
        "ar-OM-AbdullahNeural",
        "ar-OM-AyshaNeural",
        "ar-QA-AmalNeural",
        "ar-QA-MoazNeural",
        "ar-SA-HamedNeural",
        "ar-SA-ZariyahNeural",
        "ar-SY-AmanyNeural",
        "ar-SY-LaithNeural",
        "ar-TN-HediNeural",
        "ar-TN-ReemNeural",
        "ar-YE-MaryamNeural",
        "ar-YE-SalehNeural",
        "az-AZ-BabekNeural",
        "az-AZ-BanuNeural",
        "bg-BG-BorislavNeural",
        "bg-BG-KalinaNeural",
        "bn-BD-NabanitaNeural",
        "bn-BD-PradeepNeural",
        "bn-IN-BashkarNeural",
        "bn-IN-TanishaaNeural",
        "bs-BA-GoranNeural",
        "bs-BA-VesnaNeural",
        "ca-ES-AlbaNeural",
        "ca-ES-EnricNeural",
        "ca-ES-JoanaNeural",
        "cs-CZ-AntoninNeural",
        "cs-CZ-VlastaNeural",
        "cy-GB-AledNeural",
        "cy-GB-NiaNeural",
        "da-DK-ChristelNeural",
        "da-DK-JeppeNeural",
        "de-AT-IngridNeural",
        "de-AT-JonasNeural",
        "de-CH-JanNeural",
        "de-CH-LeniNeural",
        "de-DE-AmalaNeural",
        "de-DE-BerndNeural",
        "de-DE-ChristophNeural",
        "de-DE-ConradNeural",
        "de-DE-ElkeNeural",
        "de-DE-FlorianMultilingualNeural",
        "de-DE-GiselaNeural",
        "de-DE-KasperNeural",
        "de-DE-KatjaNeural",
        "de-DE-KillianNeural",
        "de-DE-KlarissaNeural",
        "de-DE-KlausNeural",
        "de-DE-LouisaNeural",
        "de-DE-MajaNeural",
        "de-DE-RalfNeural",
        "de-DE-SeraphinaMultilingualNeural",
        "de-DE-TanjaNeural",
        "el-GR-AthinaNeural",
        "el-GR-NestorasNeural",
        "en-AU-AnnetteNeural",
        "en-AU-CarlyNeural",
        "en-AU-DarrenNeural",
        "en-AU-DuncanNeural",
        "en-AU-ElsieNeural",
        "en-AU-FreyaNeural",
        "en-AU-JoanneNeural",
        "en-AU-KenNeural",
        "en-AU-KimNeural",
        "en-AU-NatashaNeural",
        "en-AU-NeilNeural",
        "en-AU-TimNeural",
        "en-AU-TinaNeural",
        "en-AU-WilliamNeural",
        "en-CA-ClaraNeural",
        "en-CA-LiamNeural",
        "en-GB-AbbiNeural",
        "en-GB-AdaMultilingualNeural",
        "en-GB-AlfieNeural",
        "en-GB-BellaNeural",
        "en-GB-ElliotNeural",
        "en-GB-EthanNeural",
        "en-GB-HollieNeural",
        "en-GB-LibbyNeural",
        "en-GB-MaisieNeural",
        "en-GB-NoahNeural",
        "en-GB-OliverNeural",
        "en-GB-OliviaNeural",
        "en-GB-OllieMultilingualNeural",
        "en-GB-RyanNeural",
        "en-GB-SoniaNeural",
        "en-GB-ThomasNeural",
        "en-HK-SamNeural",
        "en-HK-YanNeural",
        "en-IE-ConnorNeural",
        "en-IE-EmilyNeural",
        "en-IN-NeerjaNeural",
        "en-IN-PrabhatNeural",
        "en-KE-AsiliaNeural",
        "en-KE-ChilembaNeural",
        "en-NG-AbeoNeural",
        "en-NG-EzinneNeural",
        "en-NZ-MitchellNeural",
        "en-NZ-MollyNeural",
        "en-PH-JamesNeural",
        "en-PH-RosaNeural",
        "en-SG-LunaNeural",
        "en-SG-WayneNeural",
        "en-TZ-ElimuNeural",
        "en-TZ-ImaniNeural",
        "en-US-AIGenerateNeural",
        "en-US-AlloyMultilingualNeural",
        "en-US-AlloyMultilingualNeuralHD",
        "en-US-AmberNeural",
        "en-US-AnaNeural",
        "en-US-AndrewMultilingualNeural",
        "en-US-AndrewNeural",
        "en-US-AriaNeural",
        "en-US-AshleyNeural",
        "en-US-AvaMultilingualNeural",
        "en-US-AvaNeural",
        "en-US-BlueNeural",
        "en-US-BrandonNeural",
        "en-US-BrianMultilingualNeural",
        "en-US-BrianNeural",
        "en-US-ChristopherNeural",
        "en-US-CoraNeural",
        "en-US-DavisNeural",
        "en-US-EchoMultilingualNeural",
        "en-US-EchoMultilingualNeuralHD",
        "en-US-ElizabethNeural",
        "en-US-EmmaMultilingualNeural",
        "en-US-EmmaNeural",
        "en-US-EricNeural",
        "en-US-FableMultilingualNeural",
        "en-US-FableMultilingualNeuralHD",
        "en-US-GuyNeural",
        "en-US-JacobNeural",
        "en-US-JaneNeural",
        "en-US-JasonNeural",
        "en-US-JennyMultilingualNeural",
        "en-US-JennyNeural",
        "en-US-KaiNeural",
        "en-US-LunaNeural",
        "en-US-MichelleNeural",
        "en-US-MonicaNeural",
        "en-US-NancyNeural",
        "en-US-NovaMultilingualNeural",
        "en-US-NovaMultilingualNeuralHD",
        "en-US-OnyxMultilingualNeural",
        "en-US-OnyxMultilingualNeuralHD",
        "en-US-RogerNeural",
        "en-US-RyanMultilingualNeural",
        "en-US-SaraNeural",
        "en-US-ShimmerMultilingualNeural",
        "en-US-ShimmerMultilingualNeuralHD",
        "en-US-SteffanNeural",
        "en-US-TonyNeural",
        "en-ZA-LeahNeural",
        "en-ZA-LukeNeural",
        "es-AR-ElenaNeural",
        "es-AR-TomasNeural",
        "es-BO-MarceloNeural",
        "es-BO-SofiaNeural",
        "es-CL-CatalinaNeural",
        "es-CL-LorenzoNeural",
        "es-CO-GonzaloNeural",
        "es-CO-SalomeNeural",
        "es-CR-JuanNeural",
        "es-CR-MariaNeural",
        "es-CU-BelkysNeural",
        "es-CU-ManuelNeural",
        "es-DO-EmilioNeural",
        "es-DO-RamonaNeural",
        "es-EC-AndreaNeural",
        "es-EC-LuisNeural",
        "es-ES-AbrilNeural",
        "es-ES-AlvaroNeural",
        "es-ES-ArabellaMultilingualNeural",
        "es-ES-ArnauNeural",
        "es-ES-DarioNeural",
        "es-ES-EliasNeural",
        "es-ES-ElviraNeural",
        "es-ES-EstrellaNeural",
        "es-ES-IreneNeural",
        "es-ES-IsidoraMultilingualNeural",
        "es-ES-LaiaNeural",
        "es-ES-LiaNeural",
        "es-ES-NilNeural",
        "es-ES-SaulNeural",
        "es-ES-TeoNeural",
        "es-ES-TrianaNeural",
        "es-ES-VeraNeural",
        "es-ES-XimenaNeural",
        "es-GQ-JavierNeural",
        "es-GQ-TeresaNeural",
        "es-GT-AndresNeural",
        "es-GT-MartaNeural",
        "es-HN-CarlosNeural",
        "es-HN-KarlaNeural",
        "es-MX-BeatrizNeural",
        "es-MX-CandelaNeural",
        "es-MX-CarlotaNeural",
        "es-MX-CecilioNeural",
        "es-MX-DaliaNeural",
        "es-MX-GerardoNeural",
        "es-MX-JorgeNeural",
        "es-MX-LarissaNeural",
        "es-MX-LibertoNeural",
        "es-MX-LucianoNeural",
        "es-MX-MarinaNeural",
        "es-MX-NuriaNeural",
        "es-MX-PelayoNeural",
        "es-MX-RenataNeural",
        "es-MX-YagoNeural",
        "es-NI-FedericoNeural",
        "es-NI-YolandaNeural",
        "es-PA-MargaritaNeural",
        "es-PA-RobertoNeural",
        "es-PE-AlexNeural",
        "es-PE-CamilaNeural",
        "es-PR-KarinaNeural",
        "es-PR-VictorNeural",
        "es-PY-MarioNeural",
        "es-PY-TaniaNeural",
        "es-SV-LorenaNeural",
        "es-SV-RodrigoNeural",
        "es-US-AlonsoNeural",
        "es-US-PalomaNeural",
        "es-UY-MateoNeural",
        "es-UY-ValentinaNeural",
        "es-VE-PaolaNeural",
        "es-VE-SebastianNeural",
        "et-EE-AnuNeural",
        "et-EE-KertNeural",
        "eu-ES-AinhoaNeural",
        "eu-ES-AnderNeural",
        "fa-IR-DilaraNeural",
        "fa-IR-FaridNeural",
        "fi-FI-HarriNeural",
        "fi-FI-NooraNeural",
        "fi-FI-SelmaNeural",
        "fil-PH-AngeloNeural",
        "fil-PH-BlessicaNeural",
        "fr-BE-CharlineNeural",
        "fr-BE-GerardNeural",
        "fr-CA-AntoineNeural",
        "fr-CA-JeanNeural",
        "fr-CA-SylvieNeural",
        "fr-CA-ThierryNeural",
        "fr-CH-ArianeNeural",
        "fr-CH-FabriceNeural",
        "fr-FR-AlainNeural",
        "fr-FR-BrigitteNeural",
        "fr-FR-CelesteNeural",
        "fr-FR-ClaudeNeural",
        "fr-FR-CoralieNeural",
        "fr-FR-DeniseNeural",
        "fr-FR-EloiseNeural",
        "fr-FR-HenriNeural",
        "fr-FR-JacquelineNeural",
        "fr-FR-JeromeNeural",
        "fr-FR-JosephineNeural",
        "fr-FR-MauriceNeural",
        "fr-FR-RemyMultilingualNeural",
        "fr-FR-VivienneMultilingualNeural",
        "fr-FR-YvesNeural",
        "fr-FR-YvetteNeural",
        "ga-IE-ColmNeural",
        "ga-IE-OrlaNeural",
        "gl-ES-RoiNeural",
        "gl-ES-SabelaNeural",
        "gu-IN-DhwaniNeural",
        "gu-IN-NiranjanNeural",
        "he-IL-AvriNeural",
        "he-IL-HilaNeural",
        "hi-IN-MadhurNeural",
        "hi-IN-SwaraNeural",
        "hr-HR-GabrijelaNeural",
        "hr-HR-SreckoNeural",
        "hu-HU-NoemiNeural",
        "hu-HU-TamasNeural",
        "hy-AM-AnahitNeural",
        "hy-AM-HaykNeural",
        "id-ID-ArdiNeural",
        "id-ID-GadisNeural",
        "is-IS-GudrunNeural",
        "is-IS-GunnarNeural",
        "it-IT-AlessioMultilingualNeural",
        "it-IT-BenignoNeural",
        "it-IT-CalimeroNeural",
        "it-IT-CataldoNeural",
        "it-IT-DiegoNeural",
        "it-IT-ElsaNeural",
        "it-IT-FabiolaNeural",
        "it-IT-FiammaNeural",
        "it-IT-GianniNeural",
        "it-IT-GiuseppeNeural",
        "it-IT-ImeldaNeural",
        "it-IT-IrmaNeural",
        "it-IT-IsabellaMultilingualNeural",
        "it-IT-IsabellaNeural",
        "it-IT-LisandroNeural",
        "it-IT-MarcelloMultilingualNeural",
        "it-IT-PalmiraNeural",
        "it-IT-PierinaNeural",
        "it-IT-RinaldoNeural",
        "ja-JP-AoiNeural",
        "ja-JP-DaichiNeural",
        "ja-JP-KeitaNeural",
        "ja-JP-MasaruMultilingualNeural",
        "ja-JP-MayuNeural",
        "ja-JP-NanamiNeural",
        "ja-JP-NaokiNeural",
        "ja-JP-ShioriNeural",
        "jv-ID-DimasNeural",
        "jv-ID-SitiNeural",
        "ka-GE-EkaNeural",
        "ka-GE-GiorgiNeural",
        "kk-KZ-AigulNeural",
        "kk-KZ-DauletNeural",
        "km-KH-PisethNeural",
        "km-KH-SreymomNeural",
        "kn-IN-GaganNeural",
        "kn-IN-SapnaNeural",
        "ko-KR-BongJinNeural",
        "ko-KR-GookMinNeural",
        "ko-KR-HyunsuNeural",
        "ko-KR-InJoonNeural",
        "ko-KR-JiMinNeural",
        "ko-KR-SeoHyeonNeural",
        "ko-KR-SoonBokNeural",
        "ko-KR-SunHiNeural",
        "ko-KR-YuJinNeural",
        "lo-LA-ChanthavongNeural",
        "lo-LA-KeomanyNeural",
        "lt-LT-LeonasNeural",
        "lt-LT-OnaNeural",
        "lv-LV-EveritaNeural",
        "lv-LV-NilsNeural",
        "mk-MK-AleksandarNeural",
        "mk-MK-MarijaNeural",
        "ml-IN-MidhunNeural",
        "ml-IN-SobhanaNeural",
        "mn-MN-BataaNeural",
        "mn-MN-YesuiNeural",
        "mr-IN-AarohiNeural",
        "mr-IN-ManoharNeural",
        "ms-MY-OsmanNeural",
        "ms-MY-YasminNeural",
        "mt-MT-GraceNeural",
        "mt-MT-JosephNeural",
        "my-MM-NilarNeural",
        "my-MM-ThihaNeural",
        "nb-NO-FinnNeural",
        "nb-NO-IselinNeural",
        "nb-NO-PernilleNeural",
        "ne-NP-HemkalaNeural",
        "ne-NP-SagarNeural",
        "nl-BE-ArnaudNeural",
        "nl-BE-DenaNeural",
        "nl-NL-ColetteNeural",
        "nl-NL-FennaNeural",
        "nl-NL-MaartenNeural",
        "pl-PL-AgnieszkaNeural",
        "pl-PL-MarekNeural",
        "pl-PL-ZofiaNeural",
        "ps-AF-GulNawazNeural",
        "ps-AF-LatifaNeural",
        "pt-BR-AntonioNeural",
        "pt-BR-BrendaNeural",
        "pt-BR-DonatoNeural",
        "pt-BR-ElzaNeural",
        "pt-BR-FabioNeural",
        "pt-BR-FranciscaNeural",
        "pt-BR-GiovannaNeural",
        "pt-BR-HumbertoNeural",
        "pt-BR-JulioNeural",
        "pt-BR-LeilaNeural",
        "pt-BR-LeticiaNeural",
        "pt-BR-ManuelaNeural",
        "pt-BR-NicolauNeural",
        "pt-BR-ThalitaMultilingualNeural",
        "pt-BR-ThalitaNeural",
        "pt-BR-ValerioNeural",
        "pt-BR-YaraNeural",
        "pt-PT-DuarteNeural",
        "pt-PT-FernandaNeural",
        "pt-PT-RaquelNeural",
        "ro-RO-AlinaNeural",
        "ro-RO-EmilNeural",
        "ru-RU-DariyaNeural",
        "ru-RU-DmitryNeural",
        "ru-RU-SvetlanaNeural",
        "si-LK-SameeraNeural",
        "si-LK-ThiliniNeural",
        "sk-SK-LukasNeural",
        "sk-SK-ViktoriaNeural",
        "sl-SI-PetraNeural",
        "sl-SI-RokNeural",
        "so-SO-MuuseNeural",
        "so-SO-UbaxNeural",
        "sq-AL-AnilaNeural",
        "sq-AL-IlirNeural",
        "sr-Latn-RS-NicholasNeural",
        "sr-Latn-RS-SophieNeural",
        "sr-RS-NicholasNeural",
        "sr-RS-SophieNeural",
        "su-ID-JajangNeural",
        "su-ID-TutiNeural",
        "sv-SE-HilleviNeural",
        "sv-SE-MattiasNeural",
        "sv-SE-SofieNeural",
        "sw-KE-RafikiNeural",
        "sw-KE-ZuriNeural",
        "sw-TZ-DaudiNeural",
        "sw-TZ-RehemaNeural",
        "ta-IN-PallaviNeural",
        "ta-IN-ValluvarNeural",
        "ta-LK-KumarNeural",
        "ta-LK-SaranyaNeural",
        "ta-MY-KaniNeural",
        "ta-MY-SuryaNeural",
        "ta-SG-AnbuNeural",
        "ta-SG-VenbaNeural",
        "te-IN-MohanNeural",
        "te-IN-ShrutiNeural",
        "th-TH-AcharaNeural",
        "th-TH-NiwatNeural",
        "th-TH-PremwadeeNeural",
        "tr-TR-AhmetNeural",
        "tr-TR-EmelNeural",
        "uk-UA-OstapNeural",
        "uk-UA-PolinaNeural",
        "ur-IN-GulNeural",
        "ur-IN-SalmanNeural",
        "ur-PK-AsadNeural",
        "ur-PK-UzmaNeural",
        "uz-UZ-MadinaNeural",
        "uz-UZ-SardorNeural",
        "vi-VN-HoaiMyNeural",
        "vi-VN-NamMinhNeural",
        "wuu-CN-XiaotongNeural",
        "wuu-CN-YunzheNeural",
        "yue-CN-XiaoMinNeural",
        "yue-CN-YunSongNeural",
        "zh-CN-XiaochenMultilingualNeural",
        "zh-CN-XiaochenNeural",
        "zh-CN-XiaohanNeural",
        "zh-CN-XiaomengNeural",
        "zh-CN-XiaomoNeural",
        "zh-CN-XiaoqiuNeural",
        "zh-CN-XiaorouNeural",
        "zh-CN-XiaoruiNeural",
        "zh-CN-XiaoshuangNeural",
        "zh-CN-XiaoxiaoDialectsNeural",
        "zh-CN-XiaoxiaoMultilingualNeural",
        "zh-CN-XiaoxiaoNeural",
        "zh-CN-XiaoyanNeural",
        "zh-CN-XiaoyiNeural",
        "zh-CN-XiaoyouNeural",
        "zh-CN-XiaoyuMultilingualNeural",
        "zh-CN-XiaozhenNeural",
        "zh-CN-YunfengNeural",
        "zh-CN-YunhaoNeural",
        "zh-CN-YunjianNeural",
        "zh-CN-YunjieNeural",
        "zh-CN-YunxiNeural",
        "zh-CN-YunxiaNeural",
        "zh-CN-YunyangNeural",
        "zh-CN-YunyeNeural",
        "zh-CN-YunyiMultilingualNeural",
        "zh-CN-YunzeNeural",
        "zh-CN-guangxi-YunqiNeural",
        "zh-CN-henan-YundengNeural",
        "zh-CN-liaoning-XiaobeiNeural",
        "zh-CN-liaoning-YunbiaoNeural",
        "zh-CN-shaanxi-XiaoniNeural",
        "zh-CN-shandong-YunxiangNeural",
        "zh-CN-sichuan-YunxiNeural",
        "zh-HK-HiuGaaiNeural",
        "zh-HK-HiuMaanNeural",
        "zh-HK-WanLungNeural",
        "zh-TW-HsiaoChenNeural",
        "zh-TW-HsiaoYuNeural",
        "zh-TW-YunJheNeural",
        "zu-ZA-ThandoNeural",
        "zu-ZA-ThembaNeural",
    ]
    MULTILINGUAL_VOICES = [
        "de-DE-FlorianMultilingualNeural",
        "de-DE-SeraphinaMultilingualNeural",
        "en-GB-AdaMultilingualNeural",
        "en-GB-OllieMultilingualNeural",
        "en-US-AdamMultilingualNeural",
        "en-US-AlloyTurboMultilingualNeural",
        "en-US-AmandaMultilingualNeural",
        "en-US-AndrewMultilingualNeural",
        "en-US-AvaMultilingualNeural",
        "en-US-BrandonMultilingualNeural",
        "en-US-BrianMultilingualNeural",
        "en-US-ChristopherMultilingualNeural",
        "en-US-CoraMultilingualNeural",
        "en-US-DavisMultilingualNeural",
        "en-US-DerekMultilingualNeural",
        "en-US-DustinMultilingualNeural",
        "en-US-EchoTurboMultilingualNeural",
        "en-US-EmmaMultilingualNeural",
        "en-US-EvelynMultilingualNeural",
        "en-US-FableTurboMultilingualNeural",
        "en-US-LewisMultilingualNeural",
        "en-US-LolaMultilingualNeural",
        "en-US-NancyMultilingualNeural",
        "en-US-NovaTurboMultilingualNeural",
        "en-US-OnyxTurboMultilingualNeural",
        "en-US-PhoebeMultilingualNeural",
        "en-US-SamuelMultilingualNeural",
        "en-US-SerenaMultilingualNeural",
        "en-US-ShimmerTurboMultilingualNeural",
        "en-US-SteffanMultilingualNeural",
        "es-ES-ArabellaMultilingualNeural",
        "es-ES-IsidoraMultilingualNeural",
        "es-ES-TristanMultilingualNeural",
        "es-ES-XimenaMultilingualNeural",
        "fr-FR-LucienMultilingualNeural",
        "fr-FR-RemyMultilingualNeural",
        "fr-FR-VivienneMultilingualNeural",
        "it-IT-AlessioMultilingualNeural",
        "it-IT-GiuseppeMultilingualNeural",
        "it-IT-IsabellaMultilingualNeural",
        "it-IT-MarcelloMultilingualNeural",
        "ja-JP-MasaruMultilingualNeural",
        "ko-KR-HyunsuMultilingualNeural",
        "pt-BR-MacerioMultilingualNeural",
        "pt-BR-ThalitaMultilingualNeural",
        "zh-CN-XiaochenMultilingualNeural",
        "zh-CN-XiaoxiaoMultilingualNeural",
        "zh-CN-XiaoyuMultilingualNeural",
        "zh-CN-YunfanMultilingualNeural",
        "zh-CN-YunxiaoMultilingualNeural",
        "zh-CN-YunyiMultilingualNeural",
    ]
    ALL += MULTILINGUAL_VOICES
    VOICES = list(set(ALL))
    AZURE_SPEECH_KEY = os.environ.get("AZURE_SPEECH_KEY")
    AZURE_SERVICE_REGION = os.environ.get("AZURE_SERVICE_REGION")

    def __init__(self, voice, ssml=False):
        super(AzureTTS, self).__init__()
        self.voice = voice
        self.ssml = ssml
        self.parser = ActionParser(self.VENDOR)

    def wrap_speak_tag(self, text):
        """Wrap text in <speak></speak> to be legal SSML"""
        if is_ssml(text):
            return text

        text = rf"""<speak xmlns="http://www.w3.org/2001/10/synthesis" xmlns:emo="http://www.w3.org/2009/10/emotionml" xmlns:mstts="http://www.w3.org/2001/mstts" version="1.0" xml:lang="en-US">
   <voice name="{self.voice}">{text}</voice>
</speak>"""
        return text

    def word_boundary_event_callback(self, tts_data, event):
        text = tts_data.text[event.text_offset : event.text_offset + event.word_length]
        node = {"type": "word"}
        node["time"] = event.audio_offset / 1e4
        node["value"] = text
        node["start"] = event.text_offset
        node["end"] = event.text_offset + event.word_length
        tts_data.raw_nodes.append(node)
        logger.debug("Word event received: %s", node)

    def viseme_event_callback(self, tts_data, event):
        """audio_offset: The start time of each viseme, in ticks (100 nanoseconds)."""
        node = {"type": "viseme"}
        node["time"] = event.audio_offset / 1e4  # in millisecond
        node["value"] = str(event.viseme_id)
        tts_data.raw_nodes.append(node)
        logger.debug("Viseme event received: %s", node)

    def bookmark_reached_event_callback(self, tts_data, event):
        node = {"type": "ssml"}
        node["time"] = event.audio_offset / 1e4
        node["value"] = event.text
        tts_data.raw_nodes.append(node)
        logger.debug("Bookmark event received: %s", node)

    def online_tts(self, tts_data):
        """performs speech synthesis and shows the word boundary event."""
        if not self.AZURE_SPEECH_KEY:
            raise RuntimeError('"AZURE_SPEECH_KEY" was not set')
        if not self.AZURE_SERVICE_REGION:
            raise RuntimeError('"AZURE_SERVICE_REGION" was not set')

        if self.ssml:
            tts_data.text = self.wrap_speak_tag(tts_data.text)

        # Creates an instance of a speech config with specified subscription key and service region.
        speech_config = speechsdk.SpeechConfig(
            subscription=self.AZURE_SPEECH_KEY, region=self.AZURE_SERVICE_REGION
        )

        speech_config.speech_synthesis_voice_name = self.voice

        # Creates a speech synthesizer using file as audio output.
        # Replace with your own audio file name.
        with tempfile.NamedTemporaryFile(suffix=".wav") as fp:
            output = fp.name
            file_config = speechsdk.audio.AudioOutputConfig(filename=output)
            # Creates a speech synthesizer with a null output stream.
            # This means the audio output data will not be written to any output channel.
            # You can just get the audio from the result.
            # speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=None)
            speech_synthesizer = speechsdk.SpeechSynthesizer(
                speech_config=speech_config, audio_config=file_config
            )

            # Subscribes to word boundary event
            # The unit of evt.audio_offset is tick (1 tick = 100 nanoseconds), divide it by 10,000 to convert to milliseconds.
            speech_synthesizer.synthesis_word_boundary.connect(
                partial(self.word_boundary_event_callback, tts_data)
            )
            speech_synthesizer.viseme_received.connect(
                partial(self.viseme_event_callback, tts_data)
            )
            speech_synthesizer.bookmark_reached.connect(
                partial(self.bookmark_reached_event_callback, tts_data)
            )

            # Receives a text from console input and synthesizes it to result.
            if self.ssml:
                result = speech_synthesizer.speak_ssml_async(tts_data.text).get()
            else:
                result = speech_synthesizer.speak_text_async(tts_data.text).get()

            # convert audio format
            if tts_data.format == "wav":
                shutil.copy(output, tts_data.wavout)
            elif tts_data.format == "mp3":
                wave_to_mp3(output, tts_data.wavout)

            # Check result
            if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
                audio_data = result.audio_data
                logger.info("{} bytes of audio data received.".format(len(audio_data)))
            elif result.reason == speechsdk.ResultReason.Canceled:
                cancellation_details = result.cancellation_details
                logger.warning(
                    "Speech synthesis canceled: {}".format(cancellation_details.reason)
                )
                if cancellation_details.reason == speechsdk.CancellationReason.Error:
                    logger.error(
                        "Error details: {}".format(cancellation_details.error_details)
                    )
                    raise RuntimeError(
                        "Speech synthesis canceled: {} {}".format(
                            cancellation_details.reason,
                            cancellation_details.error_details,
                        )
                    )

    def align_timing(self, tts_data):
        spf = wave.open(tts_data.wavout, "r")
        signal = spf.readframes(-1)
        signal = np.fromstring(signal, np.int16)

        x = np.where(np.logical_or(signal >= 50, signal <= -50))
        max_index = x[0][-1]
        fs = spf.getframerate()
        Time = np.linspace(0, len(signal) / fs, num=len(signal))
        max_audio_duration = Time[max_index]
        spf.close()

        nodes = [node for node in tts_data.raw_nodes if node["type"] == "viseme"]
        if nodes:
            logger.debug("Before alignment: %s", nodes)
            nodes_duration = nodes[-1]["time"] / 1000.0
            ratio = max_audio_duration / nodes_duration
            for node in nodes:
                node["time"] = node["time"] * ratio
            logger.debug("After alignment: %s", nodes)

    def do_tts(self, tts_data, cache_enabled=True):
        # parse action
        backup = tts_data.text
        try:
            text = self.parser.parse(tts_data.text)
        except Exception as ex:
            text = backup
            logger.exception(ex)
        tts_data.text = text

        loaded = self.load_from_cache(tts_data) if cache_enabled else False
        if not loaded:
            self.online_tts(tts_data)

            tts_data.phonemes = self.get_phonemes(tts_data.raw_nodes)
            tts_data.markers = self.get_markers(tts_data.raw_nodes)
            tts_data.words = self.get_words(tts_data.raw_nodes)
            if cache_enabled:
                self.save_to_cache(tts_data)
                logger.info("Saved to cache")


def load_voices():
    voices = defaultdict(dict)
    for voice in AzureTTS.VOICES:
        if not voice:
            continue
        try:
            api = AzureTTS(voice=voice, ssml=True)
            voices[AzureTTS.VENDOR][voice] = api
        except Exception as ex:
            logger.exception(ex)
            break
    logger.info("Added voices: %s" % ", ".join(list(voices[AzureTTS.VENDOR].keys())))
    return voices


voices = load_voices()
