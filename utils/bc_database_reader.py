import pandas as pd

class database_reader():
    def __init__(self, filename) -> None:
        super().__init__()
        self.df = pd.read_excel(filename)


    def checkBCStatistics(self, intent_type_to_check_against):
        cnt_per_intent_type = dict()
        for i in range(len(intent_type_to_check_against)):
            cnt_per_intent_type[intent_type_to_check_against[i]] = 0


        for df_idx, df_row in self.df.iterrows():
            intent = df_row["Intent Name"]

            for i in range(len(intent_type_to_check_against)):
                if( intent_type_to_check_against[i] in intent):
                    cnt_per_intent_type[intent_type_to_check_against[i]] = cnt_per_intent_type[intent_type_to_check_against[i]] + 1
                    break
        return cnt_per_intent_type


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