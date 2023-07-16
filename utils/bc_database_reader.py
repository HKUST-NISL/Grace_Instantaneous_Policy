import pandas as pd

class database_reader():
    def __init__(self, filename) -> None:
        super().__init__()
        self.df = pd.read_excel(filename)

    def lookup_table(self, intent_name:str):
        df_line = self.df[self.df["Intent Name"] == intent_name]
        lookup_result = {}

        lookup_result['utterance'] = df_line["Utterance"].item()

        lookup_result['expressions'] = [df_line["Expressions"].item()]
        lookup_result['exp_start'] = [df_line["E_Start (%)"].item()/100]
        lookup_result['exp_end'] = [df_line["E_End (%)"].item()/100]
        lookup_result['exp_mag'] = [df_line["E_Magnitude"].item()]

        lookup_result['gestures'] = [df_line["GESTURES"].item()]
        lookup_result['ges_start'] = [df_line["G_Start (%)"].item()/100]
        lookup_result['ges_end'] = [df_line["G_End (%)"].item()/100]
        lookup_result['ges_mag'] = [df_line["G_Magnitude"].item()]

        return lookup_result