def extract_values(values, indexes):
    extracted_values = [values[i] for i in indexes if i < len(values)]
    return extracted_values

# Example usage:
values = [1, 5, 3, 8, 12]
indexes = [1, 3, 4]  # Remember Python lists are 0-indexed, so 1 refers to the second element, etc.

extracted = extract_values(values, indexes)
print(extracted)
