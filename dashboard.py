import streamlit as st
import pandas as pd
import numpy as np
from BaymaxAgent.services.language_model.pygmalion import pipe

st.title('Baymax Dashboard')

text = st.text_area('Say to Baymax')
if st.button('Submit'):
    response = pipe(text)
    st.write(response)